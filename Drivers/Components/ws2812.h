#ifndef ws2812_H_
#define ws2812_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

extern volatile uint8_t dma_waiting_ws2812;

void Set_LED (int Red, int Green, int Blue);

void WS2812_Send (void);

#endif /* ws8212_H_ */