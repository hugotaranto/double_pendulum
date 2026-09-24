#include "homing.h"

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

