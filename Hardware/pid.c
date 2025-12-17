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
    float integral_limit;   // 积分限幅
    float a;                // 一阶低通滤波系数
    float filtered_error;   // 滤波后的误差
} pid_control;

// 方向环PID控制器
static pid_control direction_pid = {
    .kp = 1.0f,
    .ki = 0.05f,
    .kd = 0.2f,
    .integral = 0.0f,
    .integral_limit = 1800.0f,
    .last_error = 0.0f,
    .output_limit = 3000.0f,
    .a = 0.3f,
    .filtered_error = 0.0f
};

// 速度环PID控制器
static pid_control speed_pid = {
    .kp = 20.0f,
    .ki = 0.15f,
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 300.0f,
    .last_error = 0.0f,
    .output_limit = 7200.0f,
    .a = 0.3f,
    .filtered_error = 0.0f
};

// 前馈控制参数结构体
typedef struct {
    float k_ff;
    float pwm_bias;
} motor_ff_t;

// 左电机前馈控制参数
static motor_ff_t left_ff  = {
    .k_ff = 34.0f,     // 初值，后面再调
    .pwm_bias = 1085.0f
};

// 右电机前馈控制参数
static motor_ff_t right_ff = {
    .k_ff = 34.0f,
    .pwm_bias = 1285.0f
};

/**
 * @brief 电机前馈控制计算函数
 * @param ff: 前馈控制参数结构体指针
 * @param target_speed: 目标速度
 * @return 前馈输出PWM值
 */
static float motor_speed_ff(motor_ff_t *ff, float target_speed)
{
    if (target_speed <= 0.0f)
        return 0.0f;

    float k_low  = ff->k_ff * 0.75f;
    float k_mid  = ff->k_ff * 0.90f;
    float k_high = ff->k_ff;

    float k;

    if (target_speed < 25.0f)
    {
        k = k_low;
    }
    else if (target_speed < 60.0f)
    {
        /* 25~60 区间线性插值 */
        float t = (target_speed - 25.0f) / (60.0f - 25.0f);
        k = k_low + t * (k_mid - k_low);
    }
    else
    {
        k = k_high;
    }

    float pwm = k * target_speed + ff->pwm_bias;

    if (pwm > speed_pid.output_limit)
        pwm = speed_pid.output_limit;

    return pwm;
}


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
 * @param target_position: 目标位置（轨迹中心为0）
 * @return 方向调整值，用于左右轮速度差
 */
int direction_control_pid(int target_position) {
    /* 获取传感器状态 */
    get_track_status();

    // 计算当前位置偏差
    uint8_t tracks = (track1 << 5) | (track2 << 4) | (track3 << 3) | (track4 << 2) | (track5 << 1) | track6;
    
    // 终点检测
    if (tracks == 63) {
        return 9999; // 特殊返回值表示终点
    }

    OLED_ShowBinNum(3, 8, tracks, 6);

    int current_position = 0;
    
    // 根据传感器加权计算位置偏差
    current_position = -5 * track1 - 3 * track2 - 1 * track3 + 1 * track4 + 3 * track5 + 5 * track6;

    //OLED_ShowSignedNum(4, 6, current_position, 4);
    
    // PID计算
    float direction_output = pid_calculate(&direction_pid, (float)target_position, (float)current_position);
    
    return (int)direction_output;
}

/**
 * @brief 速度环PID控制器
 * @param target_speed: 目标速度
 * @return 基础速度值
 */
void speed_control_pid(int target_speed, int* left_speed, int* right_speed) {
    // 获取编码器反馈速度
    left_encoder_speed = get_left_encoder_speed();
    right_encoder_speed = get_right_encoder_speed();

    printf("%d,%d\n",target_speed,(left_encoder_speed - right_encoder_speed)/2);
    OLED_ShowSignedNum(1, 3, left_encoder_speed, 4);
    OLED_ShowSignedNum(1, 11, right_encoder_speed, 4);

    /* 前馈 */
    float pwm_ff_l = motor_speed_ff(&left_ff,  (float)target_speed);
    float pwm_ff_r = motor_speed_ff(&right_ff, (float)target_speed);

    int current_speed = (left_encoder_speed+ right_encoder_speed) / 2; // 平均速度
    
    // PID计算
    float speed_output = pid_calculate(&speed_pid, (float)target_speed, (float)current_speed);
    
    /* 合成左右输出 */
    float pwm_l = pwm_ff_l + speed_output;
    float pwm_r = pwm_ff_r + speed_output;

    *left_speed  = (int)pwm_l;
    *right_speed = (int)pwm_r;
}