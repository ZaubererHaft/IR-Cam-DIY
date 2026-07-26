#include "analog.h"

extern ADC_HandleTypeDef hadc1;

HAL_StatusTypeDef Analog_PollADCData(uint16_t adc_data[3]) {
    HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
    status |= HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t new_stick_y = HAL_ADC_GetValue(&hadc1);
    adc_data[0] = new_stick_y;

    status |= HAL_ADC_Start(&hadc1);
    status |= HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t new_stick_x = HAL_ADC_GetValue(&hadc1);
    adc_data[1] = new_stick_x;

    status |= HAL_ADC_Start(&hadc1);
    status |= HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t batt_lvl = HAL_ADC_GetValue(&hadc1);
    adc_data[2] = batt_lvl;

    return status;
}
