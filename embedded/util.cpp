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

