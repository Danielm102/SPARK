#ifndef VoltageReader_H_
#define VoltageReader_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;

uint16_t readRawADC(uint32_t channel);
double readVoltage(uint32_t channel);
double readTemperature(uint32_t channel);

#endif /*VoltageReader_H_*/