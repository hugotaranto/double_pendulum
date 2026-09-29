#include "calibrate.h"
#include "AS5048A.h"

int homePendulum(double &left_lim, double &right_lim, double &center_pose, float home_speed) {

  Serial.println("Beginning Homing");
  // Homing sequence:
  // set to velocity control mode
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );

  // wait for it to switch modes properly
  delay(100);

  Serial.println("Homing to left");
  // home one way
  if (homeAxis(home_speed, left_lim) != 0) {
    // failed homing
    Serial.println("Failed Homing Left");
    return -1;
  }

  Serial.println("Homing to right");
  // home the other
  if (homeAxis(-home_speed, right_lim) != 0) {
    // failed homing
    Serial.println("Failed Homing Right");
    return -2;
  }

  // reset the lim switch flags
  left_limit_hit = false;
  right_limit_hit = false;

  Serial.print("Left pose: ");
  Serial.println(left_lim);

  Serial.print("Right pose: ");
  Serial.println(right_lim);

  center_pose = (left_lim + right_lim) / 2.0;

  Serial.print("Center pose: ");
  Serial.println(center_pose, 6);
  Serial.println("Homing Sequence Done!");

  // go back to center:
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );
  delayPump(20);

  odrv0.setPosition(center_pose);
  if (waitForPose(center_pose)) {
    Serial.println("Failed Centering!");
    return -1;
  }

  Serial.println("At center");

  return 0;
}

int homeAxis(float home_speed, double &pose) {


  // Don't home if switch is already pressed
  if (digitalRead(LIM_SWITCH_L) == HIGH) {
    Serial.println("Left switch already pressed");
    odrv0.setVelocity(0);
    return -1;
  }

  if (digitalRead(LIM_SWITCH_R) == HIGH) {
    Serial.println("Right switch already pressed");
    odrv0.setVelocity(0);
    return -1;
  }

  // Start homing
  odrv0.setVelocity(home_speed);

  // Wait for either switch
  unsigned long search_start = millis();

  while (digitalRead(LIM_SWITCH_L) == LOW &&
         digitalRead(LIM_SWITCH_R) == LOW) {

    pumpEvents(can_intf);

    // Request current data
    Get_Iq_msg_t iq_msg;
    if (!odrv0.getCurrents(iq_msg, 1000)) {
      Serial.println("Failed to obtain current from motor");
      odrv0.setVelocity(0);
      return -1;
    }

    if (abs(iq_msg.Iq_Measured) > 5.0) {
      Serial.println("Current too high while homing. Is motor obstructed?");
      odrv0.setVelocity(0);
      return -1;
    }

    if (delayPump(2)) {
      break;
    }

  }

  // A switch was hit
  Serial.println("Switch hit");

  odrv0.setVelocity(0);

  // Record encoder position
  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

  if (waitForFeedback(2000) != 0) {
    Serial.println("Failed to get feedback for pose");
    return -1;
  }

  pose = odrv0_user_data.last_feedback.Pos_Estimate;

  Serial.println("Backing off");

  // Back off
  odrv0.setVelocity(-home_speed);

  // Back off for 100 ms
  delayPump(200, false);

  Serial.println("Backed off");

  // reset the lim switches
  left_limit_hit = false;
  right_limit_hit = false;

  odrv0.setVelocity(0);
  delayPump(10);
  Serial.println("Axis Home complete\n\n");

  return 0;
}

void calibrateTorque() {

  pumpEvents(can_intf);

  // put the motor into torque mode
  odrv0.setControllerMode(
      CONTROL_MODE_TORQUE_CONTROL,
      INPUT_MODE_PASSTHROUGH
  );
  odrv0.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

  // get the current motor position
  // request feedback
  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

  // wait for the feedback
  if (waitForFeedback(10000) != 0) {
    Serial.println("Torque test failed to establish comms with xdrive");
    return;
  }

  // record the starting position of the motor
  float init_pose = odrv0_user_data.last_feedback.Pos_Estimate;

  float test_torque = 0;
  float current_step = 0.1;

  Button left_button;
  left_button.pin = 17;
  left_button.stable_state = HIGH;
  left_button.last_raw_state = HIGH;
  left_button.last_change_time = millis();
  left_button.pressed = false;

  Button right_button;
  right_button.pin = 18;
  right_button.stable_state = HIGH;
  right_button.last_raw_state = HIGH;
  right_button.last_change_time = millis();
  right_button.pressed = false;

  bool abort = false;

  int data_count = 0;

  float iq_sum = 0;

  const int NUM_SAMPLES = 100;

  // then begin the torque tests
  // both limit switches hit at the same time will cancel the test
  while(!abort) {

    pumpEvents(can_intf);

    // request feedback
    odrv0_user_data.received_feedback = false;
    odrv0.getFeedback(odrv0_user_data.last_feedback, 0);

    // update the buttons
    updateButton(left_button);
    updateButton(right_button);

    // check if both buttons are pressed
    if (right_button.pressed && left_button.pressed) {
      Serial.println("Both buttons pressed. Aborting!");
      abort = true;
      break;
    }

    // wait for the feedback
    if (waitForFeedback() != 0) {
      Serial.println("Failed to receive feedback from motor");
      abort = true;
      break;
    }

    // check that the motor has not gone out of bounds
    if (abs(init_pose - odrv0_user_data.last_feedback.Pos_Estimate) > 0.05) {
      Serial.println("Motor Span more than 0.1 turns!");
      abort = true;
      break;
    }

    pumpEvents(can_intf);

    Get_Iq_msg_t iq_msg;
    if (!odrv0.getCurrents(iq_msg, 1000)) {
      Serial.println("Failed to obtain current from motor");
      abort = true;
      break;
    }

    iq_sum += iq_msg.Iq_Measured;
    if (data_count == NUM_SAMPLES - 1) {

      pumpEvents(can_intf);

      float iq_average = iq_sum / NUM_SAMPLES;

      Serial.print("Requested motor current [A]: ");
      Serial.println(iq_msg.Iq_Setpoint);
      Serial.print("Motor current average [A]: ");
      Serial.println(iq_average);
      Serial.print("\n");

      iq_sum = 0;

      Serial.print("Requested Torque: ");
      Serial.println(test_torque);

      Serial.print("\n\n\n");
      data_count = 0;
    }

    // check for right limit switch hit to increase motor current
    if (right_button.pressed) {
      test_torque += current_step;
      right_button.pressed = false;
    }

    // check for left limit switch to increase motor current
    if (left_button.pressed) {
      test_torque -= current_step;
      left_button.pressed = false;
    }

    // command the motor torque
    odrv0.setTorque(test_torque);

    pumpEvents(can_intf);

    delayPump(2);
    data_count += 1;
  }

  odrv0.setTorque(0);

}


void staticFrictionTest(const float torque_step) {

  delayPump(2000);

  Serial.println("\n\nStarting Friction Test...");

  pumpEvents(can_intf);

  // put the xdrive into torque mode
  odrv0.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);

  odrv0.setTorque(0);

  // wait for it to change
  delayPump(100);

  // check if the switches are currently pressed
  if (digitalRead(17) == HIGH || digitalRead(18) == HIGH) {
    Serial.println("Failed friction test: Lim switch already pressed!");
    odrv0.setTorque(0);
    return;
  }

  // reset the limit switches
  right_limit_hit = false;
  left_limit_hit = false;

  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback);

  if (waitForFeedback(100) != 0) {
    Serial.println("Failed friction test: could not get initial feedback");
    odrv0.setTorque(0);
    return;
  }

  float init_pose = odrv0_user_data.last_feedback.Pos_Estimate;

  float torque_cmd = 0;
  float max_torque = 0.5;

  while(1) {

    // check lim switches
    if (right_limit_hit || left_limit_hit) {
      Serial.println("Failed friction test: Limit switch hit");
      break;
    }

    if (torque_cmd >= max_torque) {
      Serial.println("Failed friction test: max torque exceeded");
      break;
    }

    pumpEvents(can_intf);

    // set the torque
    odrv0.setTorque(torque_cmd);

    // wait a bit
    delayPump(20);

    // get feedback from the motor
    odrv0_user_data.received_feedback = false;
    odrv0.getFeedback(odrv0_user_data.last_feedback);

    if (waitForFeedback(20) != 0) {
      Serial.println("Failed friction test: could not get feedback in loop");
      odrv0.setTorque(0);
      return;
    }

    // check if it has moved
    if (abs(odrv0_user_data.last_feedback.Pos_Estimate - init_pose) > 0.01) {
      Serial.print("Cart moved at torque: ");
      Serial.println(torque_cmd, 3);
      break;
    }

    // wait a bit between increasing torque
    torque_cmd += torque_step;
    Serial.print("Increasing torque to: ");
    Serial.println(torque_cmd, 3);
    Serial.print("\n\n");

    // delayPump(1000);
    delayPump(200);

  }

  odrv0.setTorque(0);

}


void kineticFrictionTest(const float start_torque, const float torque_step, int num_steps, const float torque_start, const float home_speed) {

  // home the pendulum
  double left_lim, right_lim, center_pose;
  if (homePendulum(left_lim, right_lim, center_pose, home_speed) != 0) { 
    Serial.println("Homing failed in friction test");
    return;
  }

  delayPump(2000);

  Serial.println("\n\nStarting Friction Test...");

  pumpEvents(can_intf);

  // put the xdrive into torque mode
  odrv0.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);

  odrv0.setTorque(0);

  // wait for it to change
  delayPump(100);

  // check if the switches are currently pressed
  if (digitalRead(17) == HIGH || digitalRead(18) == HIGH) {
    Serial.println("Failed friction test: Lim switch already pressed!");
    odrv0.setTorque(0);
    return;
  }

  // reset the limit switches
  right_limit_hit = false;
  left_limit_hit = false;

  odrv0_user_data.received_feedback = false;
  odrv0.getFeedback(odrv0_user_data.last_feedback);

  if (waitForFeedback(100) != 0) {
    Serial.println("Failed friction test: could not get initial feedback");
    odrv0.setTorque(0);
    return;
  }
  
  float torque_cmd = start_torque;

  for(int i = 0; i < num_steps; i++) {

    // stop the motor
    odrv0.setTorque(0);

    // check lim switches
    if (left_limit_hit || right_limit_hit) {
      Serial.println("Failed friction test: lims hit before starting torque step");
    }

    // Move back to left
    odrv0.setControllerMode(
        ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
        ODriveInputMode::INPUT_MODE_PASSTHROUGH
    );

    if (delayPump(100)) {
      odrv0.setTorque(0);
      Serial.println("Failed friction test: Failed Delay");
      return;
    }

    odrv0.setPosition(left_lim - 0.1);
    if (waitForPose(left_lim - 0.1)) {
      Serial.println("Failed friction test: Failed pose wait");
      return;
    }

    Serial.print("Testing with torque: "); Serial.println(torque_cmd, 3);

    // set the controller mode to torque
    odrv0.setControllerMode(
        ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
        ODriveInputMode::INPUT_MODE_PASSTHROUGH
    );

    if (delayPump(50)) {
      Serial.println("Failed friction test: Failed Delay");
      odrv0.setTorque(0);
      return;
    }

    // start the torque kickoff
    Serial.println("Starting torque kickoff");
    odrv0.setTorque(-torque_start);

    if (delayPump(200)) {
      odrv0.setTorque(0);
      Serial.println("Failed friction test: Failed during torque kickoff");
      return;
    }

    // change to actual torque now that static overcome
    
    // command the given torque
    odrv0.setTorque(-torque_cmd);

    if (delayPump(500)) {
      odrv0.setTorque(0);
      Serial.println("Failed friction test: Hit lim while waiting for vel steady");
      return;
    }

    // increment the torque
    torque_cmd += torque_step;

    double vel_cum = 0;
    double current_cum = 0;

    int data_count = 0;

    unsigned long start = millis();

    while(abs(odrv0_user_data.last_feedback.Pos_Estimate - right_lim) >= 1.0) {

      if ((millis() - start) > 10000) {
        Serial.print("Timed out with torque: ");
        Serial.println(torque_cmd);
        break;
      }

      // check lim switches
      if (left_limit_hit || right_limit_hit) {
        odrv0.setTorque(0);
        Serial.println("Failed friction test: Hit lim while recording vel");
        return;
      }
      pumpEvents(can_intf);
      // request feedback if we have received the last one
      if (odrv0_user_data.received_feedback) {
        odrv0_user_data.received_feedback = false;
        odrv0.getFeedback(odrv0_user_data.last_feedback);
        vel_cum += odrv0_user_data.last_feedback.Vel_Estimate;
        data_count += 1;

        Get_Iq_msg_t iq_msg;
        if (!odrv0.getCurrents(iq_msg, 20)) {
          odrv0.setTorque(0);
          Serial.println("Failed to obtain current from motor");
          break;
        }

        current_cum += iq_msg.Iq_Measured;
      }

      delay(1);
    }

    odrv0.setTorque(0);

    if (data_count == 0) {
      Serial.println("Failed friction test: Vel count = 0");
      return;
    }

    Serial.print("Requested torque: "); Serial.println(torque_cmd);
    Serial.print("Average Velocity: "); Serial.println(vel_cum / data_count);
    Serial.print("Average Current: "); Serial.println(current_cum / data_count);
    Serial.print("\n\n\n");

  }

  odrv0.setTorque(0);

  // Set velocity to 0
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );

  delayPump(5);

  odrv0.setVelocity(0);
}


void pendulumDampingTest(AS5048A &encoder) {

  while(1) {
    signed long time = millis();
    float val = encoder.getRotationInDegrees();

    Serial.print("Time: "); Serial.print(time); 
    Serial.print(" Angle: "); Serial.println(val);

    delay(10);
  }

}

int32_t zeroEncoder(AS5048A &encoder, int num_steps) {

  const int32_t COUNTS_PER_REV = 16384;
  const int32_t HALF_REV = COUNTS_PER_REV / 2;

  int32_t previous = encoder.getRawRotation();
  int64_t sum = previous;

  for (int i = 1; i < num_steps; i++) {

    int32_t current = encoder.getRawRotation();

    int32_t diff = current - previous;

    // Unwrap across the 0/16384 boundary
    if (diff > HALF_REV) {
      diff -= COUNTS_PER_REV;
    }
    else if (diff < -HALF_REV) {
      diff += COUNTS_PER_REV;
    }

    int32_t unwrapped = previous + diff;

    sum += unwrapped;
    previous = unwrapped;

    delay(1);
  }

  int32_t average = sum / num_steps;

  // Wrap average back into [0, 16384)
  average %= COUNTS_PER_REV;

  if (average < 0) {
    average += COUNTS_PER_REV;
  }

  // encoder.setZeroPosition((uint16_t)average);
  return average;
}

