// ESP32 + TWAI + ODrive: Official example, simplified for ESP32 TWAI only
// Crappy ai code, take it with a grain of salt

#include <Arduino.h>
#include "util.h"
#include "calibrate.h"
#include "AS5048A.h"

/* ----------------- User-config ----------------- */

// as5048 stuff
#define SPI_SCK 13    // clock  = yellow
#define SPI_MISO 14   // Miso   = blue
#define SPI_MOSI 15   // Mosi   = green
#define SPI_CS 16     // CS     = white

AS5048A encoder(SPI_CS, false);

/* ------------- Target State ------------- */


/* --------------- LQR GAINS -------------- */
// friction compensation
// const double K[4] = {
//   6.32455532,
//   23.25222415,
//   3.14825042,
//   3.61609279
// };

const double TORQUE_SCALE_FACTOR = 1.2;
const double TORQUE_CONSTANT_ADD = 0.02;

const double K[4] = {
  -18.70828693, -69.53257584, -17.68010411, -13.28552712
};
const double target_state[4] = {
  0, M_PI, 0, 0
};

// const double K[4] = {
//   18.70828693, 54.90756039, 16.18778088,  7.22339001
// };
// const double target_state[4] = {
//   0, 0, 0, 0
// };

// no friction compensation
// const double K[4] = {
//   6.32455532, 21.8313651,  6.08114709,  3.83759632
// };


// friction only for shoulder joint
// const double K[4] = {
//   6.32455532, 21.53960178, 6.05013615,  3.81713687
// };


// xdrive constraints
const float HOME_SPEED = 4.0f;
const float MOVEMENT_SPEED = 15.0f;
const double MAX_TORQUE = 0.3;

/* --------------- Mechanics -------------- */

const double PULLEY_PITCH_RADIUS = 0.00637;
const double BELT_DISTANCE_PER_REV = 0.040;


/* ---------------- Encoder --------------- */

const int32_t COUNTS_PER_REV = 16384;
const int32_t HALF_REV = 8192;

int32_t last_raw = 0;
int32_t unwrapped_raw = 0;
int32_t zero_raw = 0;


/* ----------------- DEBUG ---------------- */

#define DEBUG_ITERS 50
int debug_iter = 0;

/* ----------------- Setup ---------------- */

// physical lims
double left_lim;
double right_lim;
double center_pose;

// alpha-beta filter variables
const float alpha = 0.40;
const float beta = 0.15;

float x_est_pos = 0.0;
float x_est_vel = 0.0;
bool filter_initialised = false;

// float encoder_zero = 0;

unsigned long last_encoder_read_time = 0;


/* --------------- Friction --------------- */

// Measured experimentally, motor-side
const double TAU_STATIC   = 0.0500;    // Nm, breakaway friction
// const double TAU_COULOMB  = 0.0192;    // Nm, kinetic/Coulomb friction
// const double TAU_COULOMB  = 0.005;
const double TAU_COULOMB  = 0.0005;
const double B_MOTOR      = 1.17e-3;   // Nm / (rad/s), viscous friction
//
// // Treat the motor as stationary below this speed
const double OMEGA_DEADBAND = 0.05;    // rad/s
//
// // Don't try to break static friction for tiny LQR commands
const double TAU_STATIC_TRIGGER = 0.005; // Nm

// float last_raw_angle = 0;

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
  zero_raw = zeroEncoder(encoder, 10);
  last_raw = encoder.getRawRotation();
  unwrapped_raw = last_raw;

  // setup the odrive
  setupOdrive();
  //
  // delayPump(500);
  //
  // left_limit_hit = false;
  // right_limit_hit = false;
  //
  odrv0.setLimits(MOVEMENT_SPEED, 15.0);
  // delayPump(500);

  // torque calibration test
  // calibrateTorque();

  // breakaway friction test
  // while(!left_limit_hit && !right_limit_hit) {
  //   staticFrictionTest(0.001);
  //   delayPump(2000);
  // }

  // kinetic friction test
  // kineticFrictionTest(0.025, 0.001, 5, 0.05, HOME_SPEED);

  // Home the pendulum
  // if (homePendulum(left_lim, right_lim, center_pose, HOME_SPEED) != 0) {
  //   fault = true;
  // }
  //
  // delayPump(1000);

  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback);

  waitForFeedback(100000);
  center_pose = odrv0_user_data.last_feedback.Pos_Estimate;

  // put motor into torque mode
  odrv0.setControllerMode(
    ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
    ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );


  Serial.println("Waiting for button press");
  // wait for button press
  while(!left_limit_hit && !right_limit_hit) {
    float val = encoder.getRotationInRadians();
    Serial.println(val);
    delay(10);
  }

  Serial.println("Waiting 2 seconds");
  // switch pressed
  delay(2000);

  left_limit_hit = false;
  right_limit_hit = false;

  // Pendulum damping test
  // pendulumDampingTest(encoder);

  delayPump(1000);

  Serial.println("Entering loop");
}

/* ----------------- Loop ---------------- */

void loop() {
  // movementTest(center_pose, left_lim, right_lim);

  unsigned long start_time = micros();

  pumpEvents(can_intf);

  // check if there is a fault
  if (fault) {
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

  // poll for feedback from xdrive
  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

  pumpEvents(can_intf);

  // read magnetic encoder
  unsigned long current_encoder_read_time = micros();
  uint16_t raw = encoder.getRawRotation();

  // // check for encoder errors
  String errors = encoder.getErrors();
  if (errors != "") {
    // errors encountered
    Serial.println("AS5048 Error:");
    Serial.println(errors);
  }

  // unwrap encoder position
  int32_t diff = (int32_t)raw - last_raw;

  // detect crossing of 0 / 16384 boundary
  if (diff > HALF_REV) {
    diff -= COUNTS_PER_REV;
  }
  else if (diff < -HALF_REV) {
    diff += COUNTS_PER_REV;
  }

  unwrapped_raw += diff;
  last_raw = raw;

  // convert to radians relative to zero
  float shoulder_angle = (unwrapped_raw - zero_raw) * (2.0f * M_PI / COUNTS_PER_REV);

  // check if filter is initialised
  if (!filter_initialised) {
    last_encoder_read_time = current_encoder_read_time;
    x_est_pos = shoulder_angle;
    x_est_vel = 0.0f;
    filter_initialised = true;
  }

  float dt = (current_encoder_read_time - last_encoder_read_time) / 1000000.0f;
  last_encoder_read_time = current_encoder_read_time;

  if (dt <= 0.0f) {
    dt = 0.001f;
  }

  // alpha-beta filter step
  float pred_pos = x_est_pos + (x_est_vel * dt);
  float innovation = shoulder_angle - pred_pos;

  x_est_pos = pred_pos + (alpha * innovation);
  x_est_vel = x_est_vel + ((beta / dt) * innovation);


  // wait until feedback is received from odrive
  if(waitForFeedback(900)) {
    fault = true;
    return;
  }

  // construct the state estimate:

  // get the pose/vel from the xdrive
  // x[0] = cart pose
  x[0] = (odrv0_user_data.last_feedback.Pos_Estimate - center_pose) * BELT_DISTANCE_PER_REV;

  // x[1] = shoulder_angle;
  x[1] = x_est_pos;

  // x[2] = cart vel
  x[2] = (odrv0_user_data.last_feedback.Vel_Estimate) * BELT_DISTANCE_PER_REV;

  // x[3] = shoulder angular velocity
  x[3] = x_est_vel;


  // then do LQR control


  // Serial.println("STATE:"); Serial.println(x[0], 2); Serial.println(x[1], 2);
  // Serial.println(x[2], 2); Serial.println(x[3], 2);
  // Serial.print("\n\n");

  // then do LQR
  double u = -(
    K[0] * (x[0] - target_state[0]) +
    K[1] * (x[1] - target_state[1]) +
    K[2] * (x[2] - target_state[2]) +
    K[3] * (x[3] - target_state[3])
  );

  // // convert to motor torque
  double motor_torque = u * PULLEY_PITCH_RADIUS * TORQUE_SCALE_FACTOR;
  if (motor_torque > 0) {
    motor_torque += TORQUE_CONSTANT_ADD;
  } else {
    motor_torque -= TORQUE_CONSTANT_ADD;
  }

  // double capped_torque = motor_torque;
  //
  // // motor_torque = max(motor_torque, MAX_TORQUE);
  // if (motor_torque > MAX_TORQUE) {
  //   capped_torque = MAX_TORQUE;
  // }
  // else if (motor_torque < -MAX_TORQUE) {
  //   capped_torque = -MAX_TORQUE;
  // }
  //
  // if (abs(capped_torque) < TAU_COULOMB && abs(capped_torque) > TAU_STATIC_TRIGGER) {
  //   if (capped_torque > 0) {
  //     capped_torque = TAU_COULOMB;
  //   } else {
  //     capped_torque = -TAU_COULOMB;
  //   }
  // }
  //
  // pumpEvents(can_intf);
  //
  // odrv0.setTorque(capped_torque);

  // Drake force -> motor torque
  // double tau_lqr = u * PULLEY_PITCH_RADIUS;


  // -----------------------------------------------------------------------------
  // Friction compensation
  // -----------------------------------------------------------------------------

  // ODrive velocity is in turns/s.
  // Convert to motor angular velocity in rad/s.
  // double motor_omega =
  //     odrv0_user_data.last_feedback.Vel_Estimate * 2.0 * M_PI;
  //
  // double tau_friction_comp = 0.0;
  //
  // if (fabs(motor_omega) > OMEGA_DEADBAND) {
  //
  //   // CART IS MOVING:
  //   // Compensate kinetic/Coulomb + viscous friction.
  //   //
  //   // The compensation is in the same direction as motion because
  //   // friction itself opposes the motion.
  //   tau_friction_comp =
  //       TAU_COULOMB * ((motor_omega > 0.0) ? 1.0 : -1.0)
  //       + B_MOTOR * motor_omega;
  //
  // }
  // // else {
  // //
  // //   // CART IS ESSENTIALLY STATIONARY:
  // //   // Use the direction requested by the LQR to break static friction.
  // //   if (fabs(tau_lqr) > TAU_STATIC_TRIGGER) {
  // //
  // //     tau_friction_comp =
  // //         TAU_STATIC * ((tau_lqr > 0.0) ? 1.0 : -1.0);
  // //   }
  // // }
  //
  //
  // // -----------------------------------------------------------------------------
  // // Final motor torque
  // // -----------------------------------------------------------------------------
  //
  // double motor_torque = tau_lqr + tau_friction_comp;

  // Safety saturation
  double capped_torque = constrain(
      motor_torque,
      -MAX_TORQUE,
      MAX_TORQUE
  );

  if (capped_torque != motor_torque) {
    Serial.println("Torque capped");
  }

  odrv0.setTorque(capped_torque);

  // double term_x    = K[0] * x[0];
  // double term_theta = K[1] * x[1];
  // double term_v    = K[2] * x[2];
  // double term_omega = K[3] * x[3];
  //
  // double u = -(term_x + term_theta + term_v + term_omega);
  //
  // double motor_torque = u * PULLEY_PITCH_RADIUS;
  //
  // Serial.printf(
  //   "\rX %+7.4f  TH %+7.4f  V %+7.4f  W %+7.4f"
  //   " | Xc %+7.3f Tc %+7.3f Vc %+7.3f Wc %+7.3f"
  //   " | u %+7.3f  tau %+7.4f",
  //   x[0], x[1], x[2], x[3],
  //   term_x, term_theta, term_v, term_omega,
  //   u, motor_torque
  // );
  //
  // double capped_torque = constrain(
  //     motor_torque,
  //     -MAX_TORQUE,
  //     MAX_TORQUE
  // );
  //
  // odrv0.setTorque(capped_torque);

  unsigned long loop_time = micros() - start_time;

  if (loop_time > 1000) {
    Serial.print("Loop slower than requested 1KHz. took: "); Serial.println(loop_time);
    return;
  }

  // debug_iter += 1;
  // if (debug_iter == DEBUG_ITERS) {
  //   Serial.printf(
  //     "\rSTATE: x=%+7.4f  θ=%+7.4f  v=%+7.4f  ω=%+7.4f",
  //     x[0], x[1], x[2], x[3]
  //   );
  //   // Serial.println(motor_torque, 4);
  //
  //   debug_iter = 0;
  // }

  // wait for the next tick
  while(micros() - start_time < 1000) {
    
  }

}


