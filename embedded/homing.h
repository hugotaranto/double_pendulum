#ifndef HOMING_H
#define HOMING_H

#include "util.h"

// homes the whole pendulum both directions
// returns 0 if success
int homePendulum(double &left_pose, double &right_pose, double &center_pose, float home_speed=2.0f);

// homes axis given a speed, and will update position to where limit switch is hit
// returns 0 if success
int homeAxis(float home_speed, double &pose);

// Function to calibrate motor / calculate the torque constant
// IMPORTANT: Requires motor to have torque tester arm attached
// Then test multiple current commands and evaluate force applied to scale.
void calibrateTorque();

#endif
