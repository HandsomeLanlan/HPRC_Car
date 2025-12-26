#include "control.h"
#include "tim.h"
#include "gpio.h"
#include "pid.h"
#include "HAL_OLED.h"
#include "HRSR04.h"
#include "encoder.h"

#define MAX_PWM 7200


// 设置�??标�?
volatile int target_speed = 0;
int target_position = 0;
int left_encoder_speed;
int right_encoder_speed;

int track1, track2, track3, track4, track5, track6;	//�?�??�传感器(从左往�??)
int base_speed,right_speed,left_speed,direction_adjust;

/* �?�??�循迹从左往�?? */
void get_track_status(void) {
	track1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	track2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	track3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	track4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	track5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	track6 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
}

/* PWM通道初�?�化 */
void Control_Init(void) {	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}


void SetSpeed_Left(int speed) {
    speed = -speed; //左电机�?�方向与小车前进方向相反
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
    speed = -speed; //右电机�?�方向与小车前进方向相反
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

/* 避障 */
int obstacle_avoidance(void) {
    uint32_t distance = Ultrasonic_GetDistance();   /* 获取距�?? */
    
    OLED_ShowNum(2, 10, distance, 3);

    static int flag = 0;    /* 检测到障�?�物时为1，重新回到循迹线上为0 */

    if (distance < 20) {    /* 检测到障�?�物，向左躲避直到前方无障�?�物*/
        SetSpeed_Left(0);
        SetSpeed_Right(5000);
        flag = 1;
        return 1;
    } else if (distance > 20 && flag == 1) {    /* 此时前方已经没有障�?�物 */
        SetSpeed_Left(5000);
        SetSpeed_Right(0);
    }

    get_track_status();
    uint8_t tracks = (track1 << 5) | (track2 << 4) | (track3 << 3) | (track4 << 2) | (track5 << 1) | track6;
    /* 检测到黑线，继�??�??�?? */
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
    
    /* [6�??传感器的状态]{左电机速度的�?�量,右电机速度的�?�量} */
    static const int speed_adjust[64][2] = {
        // �??列出常�?�的几�?�情况，其余�??根据实际需求补�??
        [0]  = {0, 0},      // 111111
        [7]  = {0, 0},      // 000111 - 直�??
        [14] = {-500, 500}, // 001110 - �??�??右偏，需要向左调�??
        [28] = {500, -500}, // 011100 - �??�??左偏，需要向右调�??
        [24] = {1000, -1000}, // 011000 - 左偏较�?�，需要右�??
        [63] = {0, 0}       // 111111 - 全部在线�??(十字�??口或终点)
    };
    
    // 基�?�速度
    int base_speed = 5000;
    
    // 根据传感器状态获取速度调整�??
    int left_speed = base_speed + speed_adjust[tracks][0];
    int right_speed = base_speed + speed_adjust[tracks][1];
    
    if (tracks == 63) {  // 111111
        left_speed = 0;
        right_speed = 0;
    }
    
    // 设置左右�??速度
    SetSpeed_Left(left_speed);
    SetSpeed_Right(right_speed);
}
	
#elif 1
/**
 * @brief   运�?�代�??
 */
void run(void)
{
    /* 1. 方向�??：输出差速（单位：速度�?? */
    //int diff_speed = direction_control_pid(target_position);

    /* 2. 生成左右�??标速度 */
    int target_left_speed  = target_speed;
    int target_right_speed = target_speed;

    /* 3.限制�??标速度 */
    if(target_left_speed > 7200) target_left_speed = 7200;
    if(target_left_speed < -7200) target_left_speed = -7200;
    if(target_right_speed > 7200) target_right_speed = 7200;
    if(target_right_speed < -7200) target_right_speed = -7200;

    /* 4. 读取编码�?? */
    int read_left_speed  = get_left_encoder_speed();
    int read_right_speed = get_right_encoder_speed();

    printf("%d,%d,%d\n",target_speed,read_left_speed,read_right_speed);

    /* 5. 左右速度�??分别计算 */
    //int pwm_left = speed_control_single(&speed_pid_left, &left_ff,target_left_speed,read_left_speed);
    //int pwm_right = speed_control_single(&speed_pid_right,&right_ff,target_right_speed,read_right_speed);
    int pwm_left = speed_control_single(&speed_pid_left,target_left_speed,read_left_speed);
    int pwm_right = speed_control_single(&speed_pid_right,target_right_speed,read_right_speed);
    /* 6. 输出 PWM */
    SetSpeed_Left(pwm_left);
    SetSpeed_Right(pwm_right);
}

#endif