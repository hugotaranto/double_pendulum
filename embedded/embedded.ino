// ESP32 + TWAI + ODrive: Official example, simplified for ESP32 TWAI only
// Crappy ai code, take it with a grain of salt

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>    // handmade0octopus driver
#include "ODriveCAN.h"
#include "ODriveEsp32Twai.hpp"   // provides pumpEvents(), wrap_can_intf(); requires onCanFrame()

/* ----------------- User-config ----------------- */

// CAN bus baudrate: must match all devices on the bus (official example uses 250k)
#define CAN_BAUDRATE    500000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID   0

// ESP32 TWAI pins
#define TX_GPIO_NUM     5
#define RX_GPIO_NUM     4

// Limit switches
#define LIM_SWITCH_L    17
#define LIM_SWITCH_R    18

const float HOME_SPEED = 2.0f;

/* ----------------- CAN interface ---------------- */

auto& can_intf = ESP32Can;

// Minimal CAN setup for ESP32 TWAI driver
static bool setupCan() {
  const auto kbps = CAN_BAUDRATE / 1000;
  // You can omit setPins if begin() accepts pins; kept explicit for clarity
  ESP32Can.setPins(TX_GPIO_NUM, RX_GPIO_NUM);
  ESP32Can.setRxQueueSize(16);
  ESP32Can.setTxQueueSize(16);
  return ESP32Can.begin(ESP32Can.convertSpeed(kbps), TX_GPIO_NUM, RX_GPIO_NUM);
}

/* ----------------- ODrive wiring ---------------- */

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN* odrives[] = { &odrv0 };

// Per-ODrive user data (same fields as official example)
struct ODriveUserData {
  Heartbeat_msg_t              last_heartbeat;
  bool                         received_heartbeat = false;
  Get_Encoder_Estimates_msg_t  last_feedback;
  bool                         received_feedback  = false;
} odrv0_user_data;

// Called on Heartbeat from ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* d = static_cast<ODriveUserData*>(user_data);
  d->last_heartbeat = msg;
  d->received_heartbeat = true;
}

// Called on encoder feedback from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  auto* d = static_cast<ODriveUserData*>(user_data);
  d->last_feedback = msg;
  d->received_feedback = true;
}

// Your TWAI adapter expects this hook; forward to all ODriveCAN instances
void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data) {
  for (auto* odrive : odrives) {
    odrive->onReceive(id, len, data);
  }
}

/* ----------------- Setup ---------------- */

void wait_for_pose(double pose, int time=10000) {

  unsigned long start = millis();
  while(abs(odrv0_user_data.last_feedback.Pos_Estimate - pose) > 0.05 && millis() - start < time) {
    pumpEvents(can_intf);
    // update the position
    odrv0.getFeedback(odrv0_user_data.last_feedback);
    delay(2);
  }

  // wait for the pose to fully complete
  delay_pump(20);
}

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

  double left_pose;
  // home one way
  if (homeAxis(HOME_SPEED, left_pose) != 0) {
    // failed homing
    return;
  }

  double right_pose;
  // home the other
  if (homeAxis(-HOME_SPEED, right_pose) != 0) {
    // failed homing
    return;
  }

  Serial.print("Left pose: ");
  Serial.println(left_pose);

  Serial.print("Right pose: ");
  Serial.println(right_pose);

  double center_pose = (left_pose + right_pose) / 2.0;

  Serial.print("Center pose: ");
  Serial.println(center_pose, 6);

  // switch to positional control
  odrv0.setControllerMode(
      ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
      ODriveInputMode::INPUT_MODE_PASSTHROUGH
  );

  // set limits for the vel and amps
  odrv0.setLimits(2.0, 2.0);

  delay_pump(100);

  // Command center
  odrv0.setPosition(center_pose);

  wait_for_pose(center_pose);
  delay_pump(1000);

  odrv0.setPosition(left_pose - 0.1);

  wait_for_pose(left_pose - 0.1);
  delay_pump(1000);

  odrv0.setPosition(left_pose);

  delay_pump(1000);

  odrv0.setPosition(center_pose);
  wait_for_pose(center_pose);
}

void debug_log(const char* msg) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.println(msg);
}

void debug_state(const char* msg) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.print(msg);

  Serial.print(" | L=");
  Serial.print(digitalRead(LIM_SWITCH_L));
  Serial.print(" R=");
  Serial.print(digitalRead(LIM_SWITCH_R));

  Serial.print(" | pos=");
  Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 6);

  Serial.println();
}

void delay_pump(int time_ms) {
  unsigned long start = millis();

  for (int i = 0; i < time_ms; i++) {
    pumpEvents(can_intf);
    delay(1);
  }

  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] delay_pump(");
  Serial.print(time_ms);
  Serial.print(") took ");
  Serial.print(millis() - start);
  Serial.println(" ms");
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
    delay(2);

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
  delay_pump(10);

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

  delay_pump(10);

  debug_state("HOME COMPLETE");

  Serial.println("========================================");
  Serial.println();

  return 0;
}



/* ----------------- Loop ---------------- */

void loop() {
  Serial.println("Homing Sequence Done!");
  delay(10000);
}

//
// void delay_pump(int time) {
//
//   for(int i = 0; i < time; i++) {
//     pumpEvents(can_intf);
//     delay(1);
//   }
//
// }
//
// int homeAxis(float home_speed, double &pose) {
//   Serial.println("Starting Homing...");
//
//
//   // Don't home if switch is already pressed
//   if (digitalRead(LIM_SWITCH_L) == HIGH) {
//     Serial.println("Left lim switch already pressed!");
//     odrv0.setVelocity(0);
//     return -1;
//   }
//
//   if (digitalRead(LIM_SWITCH_R) == HIGH) {
//     Serial.println("Right lim switch already pressed!");
//     odrv0.setVelocity(0);
//     return -1;
//   }
//
//   // Proceed with homing
//   odrv0.setVelocity(home_speed);
//
//   while (digitalRead(LIM_SWITCH_L) == LOW && digitalRead(LIM_SWITCH_R) == LOW) {
//     pumpEvents(can_intf);
//     delay(2);
//   }
//
//   // then a switch has been hit
//   Serial.println("Switch Hit!");
//
//   odrv0.setVelocity(0);
//   delay_pump(10);
//
//   // record the encoder position
//   odrv0.getFeedback(odrv0_user_data.last_feedback, 10);
//   pose = odrv0_user_data.last_feedback.Pos_Estimate;
//
//   // back off from the switch
//   odrv0.setVelocity(-home_speed);
//   delay_pump(100);
//
//   // stop
//   odrv0.setVelocity(0);
//
//   return 0;
// }
//
