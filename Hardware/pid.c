#include "pid.h"
#include "encoder.h"
#include "control.h"
#include "gpio.h"
#include "HAL_OLED.h"

// 方向环PID控制器
pid_control direction_pid = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 0.0f,
    .last_error = 0.0f,
    .output_limit = 7200.0f,
    .a = 0.3f,
    .filtered_error = 0.0f
};

// 速度环左电机PID控制器
pid_control speed_pid_left = {
    .kp = 334.0f,
    .ki = 1.57f,//5.8
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 400.0f,//1200
    .last_error = 0.0f,
    .output_limit = 7200.0f,
    .a = 0.4f,
    .filtered_error = 0.0f
};
// 速度环右电机PID控制器 
pid_control speed_pid_right = {
    
    .kp = 265.0f,
    .ki = 1.325f,//6.0
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 500.0f,//875
    .last_error = 0.0f,
    .output_limit = 5500.0f,
    .a = 0.4f,
    .filtered_error = 0.0f
};




/**
 * @brief PID控制器计算函数
 * @param pid: PID控制器结构体指针
 * @param expect: 期望值
 * @param actual: 真实值
 * @return PID输出值
 */
static float pid_calculate(pid_control* pid, float expect, float actual) {
    float error = expect - actual;

    /* 一阶滤波 */
    //pid->filtered_error = pid->a * error + (1 - pid->a) * pid->last_error;
    pid->filtered_error = error;

    pid->integral += pid->filtered_error;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    /* PID计算 */
    float output = pid->kp * pid->filtered_error + pid->ki * pid->integral + pid->kd * (pid->filtered_error - pid->last_error);

    /* 误差更新 */
    pid->last_error = pid->filtered_error;
    
    // 输出限幅
    if (output > pid->output_limit) 
        output = pid->output_limit;
    else if (output < -pid->output_limit) 
        output = -pid->output_limit;
    
    return output;
}

/**
 * @brief 方向环PID控制器
 * @param target_position: 目标位置，中心为0
 * @return 方向调整值，左右电机差速
 */
int direction_control_pid(int target_position) {

    // 根据传感器加权计算位置偏差
    int current_position = -5 * track1 - 3 * track2 - 1 * track3 + 1 * track4 + 3 * track5 + 5 * track6;
    
    // PID计算
    float direction_output = pid_calculate(&direction_pid, (float)target_position, (float)current_position);
    return (int)direction_output;
}


/**
 * @brief 单轮速度环PID
 * @param pid          PID控制器
 * @param target_speed 目标速度
 * @param actual_speed 实际速度
 * @return PWM 输出
 */
int speed_control_single(pid_control *pid,int target_speed,int actual_speed)
{
    /* PID 计算*/
    float pwm_pid = pid_calculate(pid,(float)target_speed,(float)actual_speed);
    return (int)pwm_pid;
}
