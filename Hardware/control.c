#include "control.h"
#include "tim.h"
#include "gpio.h"

void Control_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void SetSpeed_Left(int speed)
{
	
	
	if (speed > 7200) speed = 7200;
	if (speed < -7200) speed = -7200;
	
	if (speed > 0) {
		speed = 7201 - speed;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)speed); //PA7

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	} else {
		speed = 4000 - speed * (7200 - 4000) / 7200;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)speed); //PA7
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	}
}

void SetSpeed_Right(int speed)
{
	
	if (speed > 7200) speed = 7200;
	if (speed < -7200) speed = -7200;
	
	if (speed > 0) {
		speed = 3500 + speed * (7200 - 3500) / 7200;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); //PB0
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	} else {
		speed = 7200 + speed;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed); //PB0
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
}