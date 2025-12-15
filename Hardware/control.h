#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

extern int track1, track2, track3, track4, track5, track6;

void get_track_status(void);
void Control_Init(void);
void SetSpeed_Left(int speed);
void SetSpeed_Right(int speed);
int obstacle_avoidance(void);
void run(void);


#endif
