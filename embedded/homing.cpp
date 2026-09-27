#include "homing.h"

int homePendulum(double &left_lim, double &right_lim, double &center_pose, float home_speed) {

  // Homing sequence:
  // set to velocity control mode
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );

  // wait for it to switch modes properly
  delay(100);

  // home one way
  if (homeAxis(home_speed, left_lim) != 0) {
    // failed homing
    Serial.println("Failed Homing Left");
    return -1;
  }

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

  return 0;
}

int homeAxis(float home_speed, double &pose) {

  Serial.println();
  Serial.println("========================================");
  debug_state("HOME START");
  Serial.println("========================================");

  // Don't home if switch is already pressed
  debug_state("Checking limit switches");

  if (digitalRead(LIM_SWITCH_L) == HIGH) {
    debug_state("ERROR: Left limit switch already pressed");
    odrv0.setVelocity(0);
    return -1;
  }

  if (digitalRead(LIM_SWITCH_R) == HIGH) {
    debug_state("ERROR: Right limit switch already pressed");
    odrv0.setVelocity(0);
    return -1;
  }

  // Start homing
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] Sending velocity = ");
  Serial.println(home_speed);

  odrv0.setVelocity(home_speed);

  debug_state("Homing started");

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

    if (abs(iq_msg.Iq_Measured) > 1.0) {
      Serial.println("Current too high while homing. Is motor obstructed?");
      odrv0.setVelocity(0);
      return -1;
    }

    delayPump(2);

    // Print state every 100 ms
    static unsigned long last_debug = 0;

    if (millis() - last_debug >= 100) {
      last_debug = millis();

      Serial.print("[");
      Serial.print(millis());
      Serial.print(" ms] Searching");

      Serial.print(" | L=");
      Serial.print(digitalRead(LIM_SWITCH_L));

      Serial.print(" R=");
      Serial.print(digitalRead(LIM_SWITCH_R));

      Serial.print(" | pos=");
      Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 6);

      Serial.print(" | elapsed=");
      Serial.print(millis() - search_start);

      Serial.println(" ms");
    }
  }

  // A switch was hit
  Serial.println();
  debug_state("!!! SWITCH HIT !!!");

  Serial.print("Search took ");
  Serial.print(millis() - search_start);
  Serial.println(" ms");

  // Stop
  debug_log("Sending velocity = 0");

  odrv0.setVelocity(0);

  debug_state("STOP command sent");

  // Give the stop command some time while processing CAN
  delayPump(10);

  debug_state("After stop settling");

  // Record encoder position
  debug_log("Getting encoder position");

  odrv0.getFeedback(odrv0_user_data.last_feedback, 10);

  pose = odrv0_user_data.last_feedback.Pos_Estimate;

  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] Recorded pose = ");
  Serial.println(pose, 6);

  // Back off
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] Sending BACKOFF velocity = ");
  Serial.println(-home_speed);

  odrv0.setVelocity(-home_speed);

  debug_state("Backoff started");

  // Back off for 100 ms
  unsigned long backoff_start = millis();

  for (int i = 0; i < 100; i++) {
    pumpEvents(can_intf);
    delay(1);

    if (millis() - backoff_start >= 50 &&
        millis() - backoff_start < 51) {
      debug_state("50 ms into backoff");
    }
  }

  debug_state("Backoff finished");

  // Stop
  debug_log("Sending final velocity = 0");

  odrv0.setVelocity(0);

  delayPump(10);

  debug_state("HOME COMPLETE");

  Serial.println("========================================");
  Serial.println();

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

    // Serial.print("Set: ");
    // Serial.print(iq_msg.Iq_Setpoint, 4);
    //
    // Serial.print("  Measured: ");
    // Serial.println(iq_msg.Iq_Measured, 4);

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

