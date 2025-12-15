#include "pid.h"
#include "encoder.h"
#include "control.h"
#include "gpio.h"
#include "HAL_OLED.h"

// PID参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;         // 积分项
    float last_error;       // 上次误差
    float output_limit;     // 输出限幅
    float a;                // 一阶低通滤波系数
    float filtered_error;   // 滤波后的误差
} pid_control;

// 方向环PID控制器
static pid_control direction_pid = {
    .kp = 1.0f,
    .ki = 0.05f,
    .kd = 0.2f,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output_limit = 3000.0f,
    .a = 0.3f,
    .filtered_error = 0.0f
};

// 速度环PID控制器
static pid_control speed_pid = {
    .kp = 0.5f,
    .ki = 0.01f,
    .kd = 0.1f,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output_limit = 3000.0f,
    .a = 0.3f,
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
    pid->filtered_error = pid->a * error + (1 - pid->a) * pid->filtered_error;

    pid->integral += pid->filtered_error;
    
    // 积分限幅
    if (pid->integral > pid->output_limit)
        pid->integral = pid->output_limit;
    else if (pid->integral < -pid->output_limit)
        pid->integral = -pid->output_limit;

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
 * @param target_position: 目标位置（轨迹中心为0）
 * @return 方向调整值，用于左右轮速度差
 */
int direction_control_pid(int target_position) {
    /* 获取传感器状态 */
    get_track_status();

    // 计算当前位置偏差
    uint8_t tracks = (track1 << 5) | (track2 << 4) | (track3 << 3) | (track4 << 2) | (track5 << 1) | track6;
    
    OLED_ShowBinNum(3, 8, tracks, 6);

    int current_position = 0;
    
    // 根据传感器加权计算位置偏差
    current_position = -5 * track1 - 3 * track2 - 1 * track3 + 1 * track4 + 3 * track5 + 5 * track6;

    //OLED_ShowSignedNum(4, 6, current_position, 4);
    
    // 终点检测
    if (tracks == 63) {
        return 9999; // 特殊返回值表示终点
    }
    
    // PID计算
    float direction_output = pid_calculate(&direction_pid, (float)target_position, (float)current_position);
    
    return (int)direction_output;
}

/**
 * @brief 速度环PID控制器
 * @param target_speed: 目标速度
 * @return 基础速度值
 */
int speed_control_pid(int target_speed) {
    // 获取编码器反馈速度
    int left_encoder_speed = get_left_encoder_speed();
    int right_encoder_speed = get_right_encoder_speed();

    OLED_ShowSignedNum(1, 3, left_encoder_speed, 4);
    OLED_ShowSignedNum(1, 11, right_encoder_speed, 4);

    int current_speed = (left_encoder_speed+ right_encoder_speed) / 2; // 平均速度
    
    // PID计算
    float speed_output = pid_calculate(&speed_pid, (float)target_speed, (float)current_speed);
    
    return (int)speed_output;
}