#ifndef UTIL_H 
#define UTIL_H

#include <ESP32-TWAI-CAN.hpp>
#include "ODriveCAN.h"
#include "ODriveEsp32Twai.hpp"

// CAN bus baudrate: must match all devices on the bus (official example uses 250k)
#define CAN_BAUDRATE    500000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID   0

// ESP32 TWAI pins
#define TX_GPIO_NUM     5
#define RX_GPIO_NUM     4

// Limit switch pins
#define LIM_SWITCH_L    17
#define LIM_SWITCH_R    18


/* ----------------- CAN interface ---------------- */

extern TwaiCAN &can_intf;
bool setupCan();

/* ----------------- ODrive wiring ---------------- */

extern ODriveCAN odrv0;
extern ODriveCAN *odrives[];

// Per-ODrive user data (same fields as official example)
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
extern ODriveUserData odrv0_user_data;

void onHeartbeat(Heartbeat_msg_t &msg, void *user_data);
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data);
void onCanFrame(uint32_t id, uint8_t len, const uint8_t *data);


/* ---------------- Limit Switches ---------------- */

extern volatile bool left_limit_hit;
extern volatile bool right_limit_hit;

void IRAM_ATTR rightLimitISR();
void IRAM_ATTR leftLimitISR();

/* ----------------- General Util ----------------- */

void delayPump(int time_ms);
void waitForPose(double pose, int timeout=10000);


/* ------------------ Debugging ------------------- */
void debug_log(const char *msg);
void debug_state(const char *msg);
void movementTest(double center_pose, double left_lim, double right_lim);

#endif
