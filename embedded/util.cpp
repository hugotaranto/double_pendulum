#include "util.h"

TwaiCAN &can_intf = ESP32Can;

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN *odrives[] = { &odrv0 };

// Per-ODrive user data (same fields as official example)
ODriveUserData odrv0_user_data;

/* ----------------- CAN interface ---------------- */

// Minimal CAN setup for ESP32 TWAI driver
bool setupCan() {
  const auto kbps = CAN_BAUDRATE / 1000;
  // You can omit setPins if begin() accepts pins; kept explicit for clarity
  ESP32Can.setPins(TX_GPIO_NUM, RX_GPIO_NUM);
  ESP32Can.setRxQueueSize(16);
  ESP32Can.setTxQueueSize(16);
  return ESP32Can.begin(ESP32Can.convertSpeed(kbps), TX_GPIO_NUM, RX_GPIO_NUM);
}

/* ----------------- ODrive wiring ---------------- */

// Called on Heartbeat from ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data) {
  auto *d = static_cast<ODriveUserData*>(user_data);
  d->last_heartbeat = msg;
  d->received_heartbeat = true;
}

// Called on encoder feedback from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data) {
  auto* d = static_cast<ODriveUserData*>(user_data);
  d->last_feedback = msg;
  d->received_feedback = true;
}

// Your TWAI adapter expects this hook; forward to all ODriveCAN instances
void onCanFrame(uint32_t id, uint8_t len, const uint8_t *data) {
  for (auto* odrive : odrives) {
    odrive->onReceive(id, len, data);
  }
}

void setupOdrive() {
  
  // Odrive setup
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
}


/* ---------------- Limit Switches ---------------- */

volatile bool left_limit_hit = false;
volatile bool right_limit_hit = false;

void IRAM_ATTR leftLimitISR() {
  left_limit_hit = true;
}

void IRAM_ATTR rightLimitISR() {
  right_limit_hit = true;
}


/* ----------------- General Util ----------------- */


void delayPump(int time_ms) {
  unsigned long start = millis();

  while(millis() - start < time_ms) {
    pumpEvents(can_intf);
    delay(2);
  }

}

void waitForPose(double pose, int timeout) {

  unsigned long start = millis();
  while(abs(odrv0_user_data.last_feedback.Pos_Estimate - pose) > 0.05 && millis() - start < timeout) {
    pumpEvents(can_intf);
    // update the position
    odrv0.getFeedback(odrv0_user_data.last_feedback);
    delay(2);
  }

  // wait for the pose to fully complete
  delayPump(20);
}

int waitForFeedback(unsigned long timeout) {

  unsigned long start = micros();

  while (!odrv0_user_data.received_feedback) {
    pumpEvents(can_intf);

    if (micros() - start > timeout) {
      Serial.println("ODRIVE FEEDBACK TIMOUT");
      return -1;
    }
  }

  return 0;
}


/* ------------------ Debugging ------------------- */

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

void movementTest(double center_pose, double left_lim, double right_lim) {
  // move to the center
  odrv0.setPosition(center_pose);
  waitForPose(center_pose);
  delayPump(1000);

  // move just before the left lim
  odrv0.setPosition(left_lim - 0.1);
  waitForPose(left_lim - 0.1);
  delayPump(1000);

  // move to left lim
  odrv0.setPosition(left_lim);
  waitForPose(left_lim);
  delayPump(1000);

  // move just before right lim
  odrv0.setPosition(right_lim + 0.1);
  waitForPose(right_lim + 0.1);
  delayPump(1000);

  // move to right lim
  odrv0.setPosition(right_lim);
  waitForPose(right_lim);
  delayPump(1000);

}


/* ----------------- Button logic ----------------- */

void updateButton(Button &button) {

  bool raw_state = digitalRead(button.pin);

  // Raw state changed -> restart debounce timer
  if (raw_state != button.last_raw_state) {
    button.last_change_time = millis();
    button.last_raw_state = raw_state;
  }

  // Has the new state remained stable long enough?
  if ((millis() - button.last_change_time) >= DEBOUNCE_MS) {

    // Stable state has changed
    if (raw_state != button.stable_state) {
      button.stable_state = raw_state;

      // LOW -> HIGH = button pressed
      if (button.stable_state == HIGH) {
        button.pressed = true;
      }
    }
  }
}

