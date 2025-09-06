#include "VoltageReader.h"

/*0 => NTC1, 1 => NTC2, 2 => VD1*/
uint16_t readRawADC(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    uint16_t adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adcValue;
}

/*0 => NTC1, 1 => NTC2, 2 => VD1*/
double readVoltage(uint32_t channel) {
    double V_read = 3.3f * readRawADC(channel) / 4096.f;

    return V_read;
}

/*0 => NTC1, 1 => NTC2*/
double readTemperature(uint32_t channel) {
    uint16_t adcValue = readRawADC(channel);

    double Temperature = (double)3380/log((double)adcValue / (4096 - adcValue) * exp((double)3380/298.15)) - 273.15;

    return Temperature;
}