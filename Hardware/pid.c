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
    .kp = 225.0f,
    .ki = 5.2f,
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 1200.0f,
    .last_error = 0.0f,
    .output_limit = 7200.0f,
    .a = 0.4f,
    .filtered_error = 0.0f
};
// 速度环右电机PID控制器
pid_control speed_pid_right = {
    
    .kp = 200.0f,
    .ki = 5.4f,
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 875.0f,
    .last_error = 0.0f,
    .output_limit = 6000.0f,
    .a = 0.4f,
    .filtered_error = 0.0f
};

// // 左电机前馈控制器
// motor_ff_t left_ff  = {
//     .k_ff = 56.0f,     // 初值，后面再调
//     .pwm_bias =1085.0f
// };

// // 右电机前馈控制器
// motor_ff_t right_ff = {
//     .k_ff = 40.0f,
//     .pwm_bias = 1120.0f
// };

int battery_voltage = 12.3f; // 电池电压，全局变量，可在其他地方更新

// /**
//  * @brief 速度环前馈控制计算函数
//  * @param ff: 前馈控制参数结构体指针
//  * @param target_speed: 目标速度
//  * @return 前馈输出PWM值
//  */
// static float motor_speed_ff(motor_ff_t *ff, float target_speed)
// {
//     if (target_speed <= 0.0f)
//         return 0.0f;

//     float k_low  = ff->k_ff * 0.6f;
//     float k_mid  = ff->k_ff * 0.90f;
//     float k_high = ff->k_ff;

//     float k;

//     if (target_speed < 25.0f)
//     {
//         k = k_low;
//     }
//     else if (target_speed < 60.0f)
//     {
//         /* 25~60 区间线性插值 */
//         float t = (target_speed - 25.0f) / (60.0f - 25.0f);
//         k = k_low + t * (k_mid - k_low);
//     }
//     else
//     {
//         k = k_high;
//     }

//     float pwm = k * target_speed + ff->pwm_bias;

//     /* ===== 电池电压归一化 ===== */
//     // if (battery_voltage > 1.0f)   // 防止除零
//     // {
//     //     float voltage_scale = 11.17 / battery_voltage; // 11.17V为调参时的参考电压
//     //     pwm *= voltage_scale;
//     // }

//     return pwm;
// }



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
 * @brief 单轮速度环 PID
 * @param pid          PID控制器
 * @param ff           前馈控制器
 * @param target_speed 目标速度
 * @param actual_speed 实际速度（编码器）
 * @return PWM 输出
 */
int speed_control_single(pid_control *pid,int target_speed,int actual_speed)
{
    /* 前馈 */
    //float pwm_ff = motor_speed_ff(ff, (float)target_speed);

    /* PID 修正 */
    float pwm_pid = pid_calculate(pid,(float)target_speed,(float)actual_speed);

    //float pwm = pwm_ff + pwm_pid;
    float pwm = pwm_pid;

    return (int)pwm;
}
