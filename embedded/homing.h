#ifndef HOMING_H
#define HOMING_H

#include "util.h"

// homes axis given a speed, and will update position to where limit switch is hit
int homeAxis(float home_speed, double &pose);

#endif
