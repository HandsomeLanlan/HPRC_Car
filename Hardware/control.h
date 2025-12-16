#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"



/* 六路循迹 */
extern int track1, track2, track3, track4, track5, track6;
extern int left_speed, right_speed,target_speed;
extern int left_encoder_speed;
extern int right_encoder_speed;

void get_track_status(void);
void Control_Init(void);
void SetSpeed_Left(int speed);
void SetSpeed_Right(int speed);
int obstacle_avoidance(void);
void run(void);


#endif
