// ESP32 + TWAI + ODrive: Official example, simplified for ESP32 TWAI only
// Crappy ai code, take it with a grain of salt

#include <Arduino.h>
#include "util.h"
#include "homing.h"

/* ----------------- User-config ----------------- */

const float HOME_SPEED = 2.0f;

const float MOVEMENT_SPEED = 15.0f;

/* ----------------- Setup ---------------- */

double left_lim;
double right_lim;
double center_pose;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  // Limit switch setup
  pinMode(LIM_SWITCH_L, INPUT_PULLUP);
  pinMode(LIM_SWITCH_R, INPUT_PULLUP);

  Serial.println("Starting ODriveCAN demo (ESP32 TWAI)");

  // Register ODrive callbacks
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Init CAN
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true) { 
      delay(50);
    }
  }

  // Wait for ODrive heartbeat (pump events; add tiny yield)
  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(1);
  }
  Serial.println("Found ODrive");

  // Serial.println("Waiting for motor to boot properly");
  // delay(5000);

  // Request bus voltage/current (1s timeout)
  Serial.println("Attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1000)) {
    Serial.println("vbus request failed!");
    while (true) { delay(50); }
  }
  Serial.print("DC voltage [V]: "); Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: "); Serial.println(vbus.Bus_Current);


  // Enter CLOSED_LOOP_CONTROL with periodic event pumping (mirrors official flow)
  Serial.println("Enabling CLOSED_LOOP_CONTROL...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for ~150ms to ensure reliable state transition even on busy bus
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrive running!");


  // Homing sequence:
  // set to velocity control mode
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );

  // wait for it to switch modes properly
  delay(100);

  // home one way
  if (homeAxis(HOME_SPEED, left_lim) != 0) {
    // failed homing
    return;
  }

  // home the other
  if (homeAxis(-HOME_SPEED, right_lim) != 0) {
    // failed homing
    return;
  }

  Serial.print("Left pose: ");
  Serial.println(left_lim);

  Serial.print("Right pose: ");
  Serial.println(right_lim);

  center_pose = (left_lim + right_lim) / 2.0;

  Serial.print("Center pose: ");
  Serial.println(center_pose, 6);

  Serial.println("Homing Sequence Done!");


  // switch to positional control
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );

  // set limits for the vel and amps
  odrv0.setLimits(MOVEMENT_SPEED, 2.0);

  delayPump(100);

}

/* ----------------- Loop ---------------- */

void loop() {
  movementTest(center_pose, left_lim, right_lim);
}

