#include "encoder.h"
#include "HAL_OLED.h"
#include "tim.h"

/* 获取右电机编码器的速度 */
int get_right_encoder_speed(void) {
    static uint32_t last_time = 0;
    static short pulse_count = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每10ms读取一次脉冲数并清零
    if(current_time - last_time >= 10) {
        pulse_count = (short)__HAL_TIM_GetCounter(&htim2);
        //OLED_ShowNum(3, 1, pulse_count, 5);
        __HAL_TIM_SetCounter(&htim2, 0);
        last_time = current_time;
    }
    
    return pulse_count;  // 返回10ms内的脉冲数
}

/* 获取左电机编码器的速度 */
int get_left_encoder_speed(void) {
    static uint32_t last_time = 0;
    static short pulse_count = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每10ms读取一次脉冲数并清零
    if(current_time - last_time >= 10) {
        pulse_count = (short)__HAL_TIM_GetCounter(&htim4);
        //OLED_ShowNum(3, 1, pulse_count, 5);
        __HAL_TIM_SetCounter(&htim4, 0);
        last_time = current_time;
    }
    
    return -pulse_count;  // 返回10ms内的脉冲数
}