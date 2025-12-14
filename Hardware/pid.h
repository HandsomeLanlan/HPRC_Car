#ifndef __PID_H
#define __PID_H

#include "main.h"

int direction_control_pid(int target_position);
int speed_control_pid(int target_speed);

#endif