#include "control.h"

PendulumController::PendulumController(
    AS5048A &encoder,
    const double K[4],
    const double target_state[4],
    Mode mode,
    float max_torque,
    float max_velocity
) : encoder(encoder) {

  for (int i = 0; i < 4; i++) {
    this->K[i] = K[i];
    this->target_state[i] = target_state[i];
  }

  this->fault = false;
  this->mode = mode;
  this->max_torque = max_torque;
  this->max_velocity = max_velocity;

  this->encoder_filter_initialised = false;
  this->x_est_pos = 0;
  this->x_est_vel = 0;
  this->last_encoder_read_time = 0;

}

void PendulumController::calibrate(float center_pose, float zero_raw, float last_raw, float unwrapped_raw) {

  this->center_pose = center_pose;
  this->zero_raw = zero_raw;
  this->last_raw = last_raw;
  this->unwrapped_raw = unwrapped_raw;

  if (this->mode == Mode::FORCE) {
    odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
    );
  } else if (this->mode == Mode::VELOCITY) {
    odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
    );
  }

}

double PendulumController::iterate() {

  // poll for feedback from xdrive
  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

  pumpEvents(can_intf);

  // get data from the magnetic encoder
  getEncoderState();

  // get data from odrive
  getOdrvState();

  // compute the force output
  return computeOutput();

}

void PendulumController::getEncoderState() {

  unsigned long current_encoder_read_time = micros();
  uint16_t raw = encoder.getRawRotation();

  // check for encoder errors
  String errors = encoder.getErrors();
  if (errors != "") {
    // errors encountered
    Serial.println("AS5048 Error:");
    Serial.println(errors);
  }

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
  if (!encoder_filter_initialised) {
    last_encoder_read_time = current_encoder_read_time;
    x_est_pos = shoulder_angle;
    x_est_vel = 0.0f;
    encoder_filter_initialised = true;
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

  x[1] = x_est_pos;
  x[3] = x_est_vel;

}

void PendulumController::getOdrvState() {

  if(waitForFeedback(900)) {
    Serial.println("Failed to get feedback from odrive");
    this->fault = true;    
  }

  x[0] = (odrv0_user_data.last_feedback.Pos_Estimate - center_pose) * BELT_DISTANCE_PER_REV;
  x[2] = (odrv0_user_data.last_feedback.Vel_Estimate) * BELT_DISTANCE_PER_REV;

}

double PendulumController::computeOutput() {

  double u = -(
    K[0] * (x[0] - target_state[0]) +
    K[1] * (x[1] - target_state[1]) +
    K[2] * (x[2] - target_state[2]) +
    K[3] * (x[3] - target_state[3])
  );

  if (this->mode == Mode::FORCE) {

    // convert to motor torque
    double motor_torque = u * PULLEY_PITCH_RADIUS * TORQUE_SCALE_FACTOR;
    if (motor_torque > 0) {
      motor_torque += TORQUE_CONSTANT_ADD;
    } else if (motor_torque < 0) {
      motor_torque -= TORQUE_CONSTANT_ADD;
    }

    // Safety saturation
    double capped_torque = constrain(
        motor_torque,
        -this->max_torque,
        this->max_torque
    );

    if (capped_torque != motor_torque) {
      // Serial.println("Torque capped from: ");
      Serial.printf("Torque capped from: %f\n", motor_torque);
    }

    return capped_torque;

  } else if (this->mode == Mode::VELOCITY) {

    // return u;
    double capped_velocity = constrain(
        u,
        -max_velocity,
        max_velocity
    );

    if (capped_velocity != u) {
      Serial.println("Velocity capped");
    }

    return capped_velocity;

  }

  return 0;

}

bool PendulumController::getError() {
  return this->fault;
}

void PendulumController::setInput(double input) {

  if (this->mode == Mode::FORCE) {
    odrv0.setTorque(input);
  } else if (this->mode == Mode::VELOCITY) {
    odrv0.setVelocity(input);
  }

}
