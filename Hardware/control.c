#include "control.h"
#include "tim.h"
#include "gpio.h"

void Control_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void SetSpeed_Left(int speed)
{
	
	
	if (speed > 7200) speed = 7200;
	if (speed < -7200) speed = -7200;
	
	if (speed > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)speed); //PA7
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)-speed); //PA7
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	}
}

void SetSpeed_Right(int speed)
{
	
	if (speed > 7200) speed = 7200;
	if (speed < -7200) speed = -7200;
	
	if (speed > 0) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed); //PA2
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	} else {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -speed); //PA2
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	}
}