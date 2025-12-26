#include "control.h"
#include "tim.h"
#include "gpio.h"
#include "pid.h"
#include "HAL_OLED.h"
#include "HRSR04.h"
#include "encoder.h"
#define MAX_PWM 7200

// 设置目标速度
volatile int target_speed = 0;

int target_position = 0;
short read_left_speed = 0;
short read_right_speed = 0;

int track1, track2, track3, track4, track5, track6;	
uint32_t distance;

/**
 * @brief   主控制函数
 */
void run(void)
{
    /* 获取传感器状态 */
    get_track_status();

    /* 方向环：输出差速 */
    int diff_speed = direction_control_pid(target_position);

    /* 生成左右电机目标速度 */
    int target_left_speed  = target_speed + diff_speed;
    int target_right_speed = target_speed - diff_speed;

    /* 读取左右电机实际速度 */
    read_left_speed  = get_left_encoder_speed();
    read_right_speed = get_right_encoder_speed();

    /* 左右速度环分别计算 */
    int pwm_left = speed_control_single(&speed_pid_left,target_left_speed,read_left_speed);
    int pwm_right = speed_control_single(&speed_pid_right,target_right_speed,read_right_speed);

    /* 输出 PWM */
    SetSpeed_Left(pwm_left);
    SetSpeed_Right(pwm_right);
}

/**
 * @brief 六路循迹从左往右读
 * @retval 返回六路循迹的状态值，返回63时到达终点
 */
uint8_t get_track_status(void) {
	track1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	track2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	track3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	track4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	track5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	track6 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

    // 计算当前位置偏差
    uint8_t tracks = (track1 << 5) | (track2 << 4) | (track3 << 3) | (track4 << 2) | (track5 << 1) | track6;
    return tracks;
}

/**
 *  @brief 避障控制函数
 *  @retval 返回1表示还未完成避障任务，返回0表示已经完成避障任务
 */
uint8_t obstacle_avoidance(void) {

    static int flag = 0;    /* 检测到障碍物时为1，重新回到循迹线上为0 */

    distance = Ultrasonic_GetDistance();
    if (distance < 20) {    /* 检测到障碍物，向左躲避直到前方无障碍物*/
        SetSpeed_Left(0);
        SetSpeed_Right(5000);
        flag = 1;
    } else if (distance > 20 && flag == 1) {    /* 此时前方已经没有障碍物 */
        SetSpeed_Left(5000);
        SetSpeed_Right(0);
    }

    uint8_t tracks = get_track_status();
    /* 检测到黑线，继续循迹*/
    if (tracks != 0 && tracks != 63) {
        flag = 0;
        return 0; //完成避障任务，返回0
    }
    return 1; 
}

/* PWM通道初始化 */
void Control_Init(void) {	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void SetSpeed_Left(int speed) {
    speed = -speed; 
    if(speed > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MAX_PWM - speed);
    }
    else
    {   
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_PWM + speed); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MAX_PWM);         
    }
}

void SetSpeed_Right(int speed) {
    speed = -speed; 
    if(speed > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MAX_PWM - speed);
    }
    else
    {   
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MAX_PWM + speed); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MAX_PWM);         
    }
}
