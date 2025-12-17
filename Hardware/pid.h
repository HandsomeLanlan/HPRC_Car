#ifndef __PID_H
#define __PID_H

#include "main.h"

int direction_control_pid(int target_position);
void speed_control_pid(int target_speed, int* left_speed, int* right_speed);

#endif