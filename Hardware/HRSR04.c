#include "HRSR04.h"

volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
volatile uint8_t capture_done = 0;
volatile uint8_t is_first_capture = 1;

void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

void Ultrasonic_Trigger(void)
{
    capture_done = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    //HAL_Delay(15);ms
    Delay_us(15);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

/* 距离计算函数 */
uint32_t Ultrasonic_GetDistance(void)
{
    Ultrasonic_Trigger();

    uint32_t res = 0;
    if(pulse_end - pulse_start > 11600) //11600 / 58 = 200cm , 超过200cm视为无效
        res = 0;
    else
        res = (pulse_end - pulse_start) / 58;  // cm

    return res;
}

