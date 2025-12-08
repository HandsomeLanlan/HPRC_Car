#include "badc.h"

double adc_getvalue(ADC_HandleTypeDef *hadc1) {
    HAL_ADC_Start(hadc1);
    return HAL_ADC_GetValue(hadc1) * 12.0 / 4096;
}