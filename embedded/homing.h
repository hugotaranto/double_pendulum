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

// Function to test cart static friction
// Experimentally averaged breakaway friction with 0.05 Nm motor torque
void staticFrictionTest(const float torque_step=0.01);

// Function to test cart kinetic friction
// results in excel
// coulomb / kinetic friction = 0.0192 Nm
// viscous friction b = 1.17 * 10^-3
void kineticFrictionTest(const float start_torque, const float torque_step=0.1, int num_steps=5, const float torque_start=0.1, const float home_speed=2.0);

#endif
