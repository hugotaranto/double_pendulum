// ESP32 + TWAI + ODrive: Official example, simplified for ESP32 TWAI only
// Crappy ai code, take it with a grain of salt

#include <Arduino.h>
#include "util.h"
#include "homing.h"
#include "AS5048A.h"

/* ----------------- User-config ----------------- */

// as5048 stuff
#define SPI_SCK 13    // clock  = yellow
#define SPI_MISO 14   // Miso   = blue
#define SPI_MOSI 15   // Mosi   = green
#define SPI_CS 16     // CS     = white

AS5048A encoder(SPI_CS, false);

// xdrive constraints
const float HOME_SPEED = 2.0f;
const float MOVEMENT_SPEED = 15.0f;

/* ----------------- Setup ---------------- */

double left_lim;
double right_lim;
double center_pose;

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

  // setup the odrive
  setupOdrive();

  // torque calibration test
  calibrateTorque();

  // Home the pendulum
  // if (homePendulum(left_lim, right_lim, center_pose, HOME_SPEED) != 0) {
  //   fault = true;
  // }

  // switch to positional control
  // odrv0.setControllerMode(
  //     ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
  //     ODriveInputMode::INPUT_MODE_PASSTHROUGH
  // );

  // set limits for the vel and amps
  odrv0.setLimits(MOVEMENT_SPEED, 15.0);

  delayPump(100);
}

/* ----------------- Loop ---------------- */

void loop() {
  // movementTest(center_pose, left_lim, right_lim);

  pumpEvents(can_intf);

  // check if there is a fault
  if (fault) {
    odrv0.setVelocity(0);
    delay(100);
    return;
  }

  // check if any of the limit switches have been hit:
  if (left_limit_hit || right_limit_hit) {
    odrv0.setVelocity(0);
    Serial.println("STOPPED FOR LIM SWITCH");
    fault = true;
    return;
  }

  // poll for feedback from xdrive
  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

  // get magnetic encoder position
  float val = encoder.getRotationInRadians();
  // check for encoder errors
  String errors = encoder.getErrors();
  if (errors != "") {
    // errors encountered
    Serial.println("AS5048 Error:");
    Serial.println(errors);
  }

  // wait until feedback is received from odrive
  // unsigned long start = micros();
  // while (!odrv0_user_data.received_feedback) {
  //   pumpEvents(can_intf);
  //
  //   if (micros() - start > 1000) {
  //     fault = true;
  //     Serial.println("ODRIVE FEEDBACK TIMOUT");
  //     return;
  //   }
  // }

  if (waitForFeedback() != 0) {
    fault = true;
    return;
  }

  // then do LQR

}

