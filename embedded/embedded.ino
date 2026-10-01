// ESP32 + TWAI + ODrive: Official example, simplified for ESP32 TWAI only
// Crappy ai code, take it with a grain of salt

#include <Arduino.h>
#include "util.h"
#include "calibrate.h"
#include "AS5048A.h"
#include "control.h"

/* ----------------- User-config ----------------- */

// as5048 stuff
#define SPI_SCK 13    // clock  = yellow
#define SPI_MISO 14   // Miso   = blue
#define SPI_MOSI 15   // Mosi   = green
#define SPI_CS 16     // CS     = white

AS5048A encoder(SPI_CS, false);


/* --------------- LQR GAINS -------------- */

const double K[4] = {
  -18.70828693, -69.53257584, -17.68010411, -13.28552712
};


/* ------------- Target State ------------- */

const double target_state[4] = {
  0, M_PI, 0, 0
};

// const double target_state[4] = {
//   0, 0, 0, 0
// };

// xdrive constraints
const float HOME_SPEED = 4.0f;
// const float MOVEMENT_SPEED = 15.0f;
const double MAX_TORQUE = 2.0;
const double MAX_CURRENT = 15;
const double MAX_VELOCITY = 15;

/* -------------- Controller -------------- */
PendulumController::Mode mode = PendulumController::Mode::FORCE;

PendulumController controller(encoder, K, target_state, mode, MAX_TORQUE, MAX_VELOCITY);

/* ----------------- DEBUG ---------------- */

#define DEBUG_ITERS 50
int debug_iter = 0;

/* ----------------- Setup ---------------- */

// physical lims
double left_lim;
double right_lim;
double center_pose;

unsigned long last_encoder_read_time = 0;

// state vector [cart pose, shoulder angle (rads), cart vel (m/s), shoulder angular velocity (rad/s)]
double x[4];

bool fault = false;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  // Limit switch setup
  pinMode(LIM_SWITCH_L, INPUT_PULLUP);
  pinMode(LIM_SWITCH_R, INPUT_PULLUP);

  // attach the interrupts
  attachInterrupt(
      digitalPinToInterrupt(LIM_SWITCH_L),
      leftLimitISR,
      RISING
  );

  attachInterrupt(
      digitalPinToInterrupt(LIM_SWITCH_R),
      rightLimitISR,
      RISING
  );

  // as5048 magnetic encoder setup
  Serial.println("Starting AS5048A encoder...");
  encoder.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // zero the encoder
  float zero_raw = zeroEncoder(encoder, 10);
  float last_raw = encoder.getRawRotation();
  float unwrapped_raw = last_raw;

  // setup the odrive
  setupOdrive();
  odrv0.setLimits(MAX_VELOCITY, MAX_TORQUE);
  // odrv0.setLimits(MAX_VELOCITY, MAX_CURRENT);
  // odrv0.setLimits(float velocity_limit, float current_soft_max)

  // get odrv feedback
  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

  waitForFeedback(1000);
  center_pose = odrv0_user_data.last_feedback.Pos_Estimate;

  Serial.print("Center Pose: "); Serial.println(center_pose);

  // calibrate the controller
  controller.calibrate(center_pose, zero_raw, last_raw, unwrapped_raw);

  // wait for button press
  Serial.println("Waiting for button press");
  while(!left_limit_hit && !right_limit_hit) {
    float val = encoder.getRotationInRadians();

    odrv0_user_data.received_feedback = false;
    odrv0.getFeedback(odrv0_user_data.last_feedback, 0);
    if (waitForFeedback(1000)) {
      Serial.println("Failed comms");
    }

    Serial.printf("Encoder Val: %f || Cart Pose: %f\n", val, odrv0_user_data.last_feedback.Pos_Estimate);
    delay(500);
  }

  delayPump(2000, false);

  left_limit_hit = false;
  right_limit_hit = false;


  Serial.println("Entering loop");
}

/* ----------------- Loop ---------------- */

void loop() {
  // movementTest(center_pose, left_lim, right_lim);

  unsigned long start_time = micros();

  pumpEvents(can_intf);

  // check if there is a fault
  if (fault || controller.getError()) {
    // make sure torque setpoint is 0
    odrv0.setTorque(0);

    // change to velocity mode
    odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
        ODriveInputMode::INPUT_MODE_PASSTHROUGH
    );
    delayPump(10);
    odrv0.setVelocity(0);

    delayPump(100);
    return;
  }

  // check if any of the limit switches have been hit:
  if (left_limit_hit || right_limit_hit) {
    odrv0.setVelocity(0);
    Serial.println("STOPPED FOR LIM SWITCH");
    fault = true;
    return;
  }

  double odrv_input = controller.iterate();

  // odrv0.setTorque(odrv_input);
  controller.setInput(odrv_input);

  // if (debug_iter == DEBUG_ITERS) {
  //   Serial.println(odrv_input);
  //   debug_iter = 0;
  // }
  // debug_iter++;

  unsigned long loop_time = micros() - start_time;

  if (loop_time > 1000) {
    Serial.print("Loop slower than requested 1KHz. took: "); Serial.println(loop_time);
    return;
  }

  // wait for the next tick
  while(micros() - start_time < 2000) {
  }

}


