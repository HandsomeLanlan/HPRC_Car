#include "control.h"
#include "tim.h"
#include "gpio.h"
#include "pid.h"
#include "HAL_OLED.h"
#include "HRSR04.h"

#define min_left_speed 3500		//左电机小于3500不转
#define min_right_speed 3500	//右电机小于3500不转

// 设置目标值
int target_speed = 20;
int target_position = 0;
int left_encoder_speed;
int right_encoder_speed;

int track1, track2, track3, track4, track5, track6;	//红外传感器(从左往右)
int base_speed,right_speed,left_speed,direction_adjust;

/* 红外循迹从左往右 */
void get_track_status(void) {
	track1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	track2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	track3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	track4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	track5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	track6 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
}

/* PWM通道初始化 */
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
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); 	//PB0
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);		//PB1
	} else {
		speed = -speed;
		speed = min_right_speed + speed * (7200 - min_right_speed) / 7200;	//小于3500不转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); 		//PB0
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);	//PB1
	}
}

/* 避障 */
int obstacle_avoidance(void) {
    uint32_t distance = Ultrasonic_GetDistance();   /* 获取距离 */
    
    OLED_ShowNum(2, 10, distance, 3);

    static int flag = 0;    /* 检测到障碍物时为1，重新回到循迹线上为0 */

    if (distance < 20) {    /* 检测到障碍物，向左躲避直到前方无障碍物*/
        SetSpeed_Left(0);
        SetSpeed_Right(5000);
        flag = 1;
        return 1;
    } else if (distance > 20 && flag == 1) {    /* 此时前方已经没有障碍物 */
        SetSpeed_Left(5000);
        SetSpeed_Right(0);
    }

    get_track_status();
    uint8_t tracks = (track1 << 5) | (track2 << 4) | (track3 << 3) | (track4 << 2) | (track5 << 1) | track6;
    /* 检测到黑线，继续循迹 */
    if (tracks) {
        flag = 0;
        return 0;
    }
    return 0;
}

/* 整体的逻辑代码 */
#if 0
void run(void) {
    uint8_t tracks = (track1 << 5) | (track2 << 4) | (track3 << 3) | 
                          (track4 << 2) | (track5 << 1) | track6;
    
    /* [6个传感器的状态]{左电机速度的增量,右电机速度的增量} */
    static const int speed_adjust[64][2] = {
        // 只列出常见的几种情况，其余可根据实际需求补充
        [0]  = {0, 0},      // 111111
        [7]  = {0, 0},      // 000111 - 直行
        [14] = {-500, 500}, // 001110 - 微微右偏，需要向左调整
        [28] = {500, -500}, // 011100 - 微微左偏，需要向右调整
        [24] = {1000, -1000}, // 011000 - 左偏较多，需要右转
        [63] = {0, 0}       // 111111 - 全部在线上(十字路口或终点)
    };
    
    // 基础速度
    int base_speed = 5000;
    
    // 根据传感器状态获取速度调整值
    int left_speed = base_speed + speed_adjust[tracks][0];
    int right_speed = base_speed + speed_adjust[tracks][1];
    
    if (tracks == 63) {  // 111111
        left_speed = 0;
        right_speed = 0;
    }
    
    // 设置左右轮速度
    SetSpeed_Left(left_speed);
    SetSpeed_Right(right_speed);
}
	
#elif 1
/**
 * @brief   运行代码
 */
void run(void) {
	/*  避障代码 */
    // int statue = obstacle_avoidance();
	// if (statue)
    //     return;
    
    // 调用速度环PID控制器
    speed_control_pid(target_speed,&left_speed,&right_speed);
    
    // 调用方向环PID控制器
    direction_adjust = direction_control_pid(target_position);
    
    //OLED_ShowSignedNum(4, 1, base_speed, 4);
    //OLED_ShowSignedNum(4, 7, direction_adjust, 4);

    // 检查是否到达终点
    // if (direction_adjust == 9999) {
    //     SetSpeed_Left(0);
    //     SetSpeed_Right(0);
    //     return;
    // }
    
    // 计算左右轮速度
    direction_adjust = 0; //先调速度环，忽略方向环
    left_speed = left_speed - direction_adjust;
    right_speed = right_speed + direction_adjust;
    
    // 限制速度范围
    if (left_speed > 7200) left_speed = 7200;
    if (left_speed < -7200) left_speed = -7200;
    if (right_speed > 7200) right_speed = 7200;
    if (right_speed < -7200) right_speed = -7200;
    
    // 设置左右轮速度
    SetSpeed_Left(left_speed);
    SetSpeed_Right(right_speed);

    OLED_ShowSignedNum(4, 1, left_speed, 4);
    OLED_ShowSignedNum(4, 7, right_speed, 4);
}
#endif