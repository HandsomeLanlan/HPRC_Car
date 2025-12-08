#ifndef __HRSR04_H
#define __HRSR04_H

#include "main.h"

#define HRSR04_W_TR(x)  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)(x))
#define HRSR04_W_EC(x)  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, (GPIO_PinState)(x))

extern volatile uint32_t pulse_start;
extern volatile uint32_t pulse_end;
extern volatile uint8_t capture_done;
extern volatile uint8_t is_first_capture;

void Delay_us(uint32_t xus);
void Ultrasonic_Trigger(void);
uint32_t Ultrasonic_GetDistance(void);

#endif
