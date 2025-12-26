#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

extern int track1, track2, track3, track4, track5, track6;
extern volatile int target_speed;
extern short read_left_speed, read_right_speed;
extern uint32_t distance;

uint8_t get_track_status(void);
void Control_Init(void);
void SetSpeed_Left(int speed);
void SetSpeed_Right(int speed);
uint8_t obstacle_avoidance(void);
void run(void);

#endif
