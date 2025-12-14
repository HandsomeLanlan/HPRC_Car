#include "control.h"
#include "tim.h"
#include "gpio.h"

#define min_left_speed 3500		//左电机小于3500不转
#define min_right_speed 3500	//右电机小于3500不转

void Control_Init(void) {	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}


void SetSpeed_Left(int speed) {
	
	
	if (speed > 7200) speed = 7200;
	if (speed < -7200) speed = -7200;
	
	if (speed > 0) {
		speed = min_left_speed + speed * (7200 - min_left_speed) / 7200;	//小于3500不转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); 	//PA6
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);		//PA7
	} else {
		speed = -speed;
		speed = min_left_speed + speed * (7200 - min_left_speed) / 7200;	//小于3500不转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); 		//PA6
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);	//PA7
	}
}

void SetSpeed_Right(int speed) {
	
	if (speed > 7200) speed = 7200;
	if (speed < -7200) speed = -7200;
	
	if (speed > 0) {
		speed = min_right_speed + speed * (7200 - min_right_speed) / 7200;	//小于3500不转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); 	//PB0
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);		//PB1
	} else {
		speed = -speed;
		speed = min_right_speed + speed * (7200 - min_right_speed) / 7200;	//小于3500不转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); 		//PB0
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);	//PB1
	}
}