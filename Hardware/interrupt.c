#include "interrupt.h"

// 在全局区域定义
volatile uint8_t mpu6050_data_ready = 0; // ‘volatile’ 关键字很重要，防止编译器优化

/**
 * @brief  超声波定时器1中断回调函数
 * @param  htim: 定时器句柄
 * @retval 无
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	if(TIM1 == htim->Instance)
	{
		if (is_first_capture)
    {
        pulse_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        is_first_capture = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING); // 改为下降沿
    }
    else
    {
        pulse_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        capture_done = 1;
        is_first_capture = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING); // 准备下次捕获
    }
	}
}

#if 0
/* MPU6050中断回调函数 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MPU6050_INT_Pin) // 检查是否是MPU6050的INT引脚产生的中断
    {
        // 1. 清除MPU6050的中断状态位（非常重要！）
        // 这通常通过读取MPU6050的某个寄存器来完成，例如：
        // uint8_t int_status = MPU6050_Read_Byte(INT_STATUS_REG);
        // (具体函数取决于你使用的驱动库)
        
        // 2. 设置事件标志，通知主循环
        mpu6050_data_ready = 1;
    }
}
#else

#endif
