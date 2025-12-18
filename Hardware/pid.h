#ifndef __PID_H
#define __PID_H

#include "main.h"


// PID参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;         // 积分项
    float last_error;       // 上次误差
    float output_limit;     // 输出限幅
    float integral_limit;   // 积分限幅
    float a;                // 一阶低通滤波系数
    float filtered_error;   // 滤波后的误差
} pid_control;

// 前馈控制参数结构体
typedef struct {
    float k_ff;
    float pwm_bias;
} motor_ff_t;

int direction_control_pid(int target_position);
int speed_control_single(pid_control *pid,int target_speed,int actual_speed);
extern motor_ff_t left_ff;
extern motor_ff_t right_ff;
extern pid_control speed_pid_right;
extern pid_control speed_pid_left;
extern int battery_voltage;

#endif