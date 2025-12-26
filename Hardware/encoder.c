#include "encoder.h"
#include "tim.h"

/* 获取右电机编码器的速度 */
short get_right_encoder_speed(void) {

    short pulse_count = 0; 
    pulse_count = (short)__HAL_TIM_GetCounter(&htim2);
    __HAL_TIM_SetCounter(&htim2, 0);
    return -pulse_count;
}

/* 获取左电机编码器的速度 */
short get_left_encoder_speed(void) {
    short pulse_count = 0;  
    pulse_count = (short)__HAL_TIM_GetCounter(&htim4);
    __HAL_TIM_SetCounter(&htim4, 0);
    return pulse_count;
}