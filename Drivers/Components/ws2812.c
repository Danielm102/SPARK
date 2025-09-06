#include "ws2812.h"

uint8_t LED_Data[3];

void Set_LED (int Red, int Green, int Blue)
{
	LED_Data[0] = Green;
	LED_Data[1] = Red;
	LED_Data[2] = Blue;
}

uint8_t pwmData_RGB[24+50] = { 0 };

void WS2812_Send (void)
{
	if (dma_waiting_ws2812)
		return; // If DMA is already waiting, do not start a new transfer

	uint8_t indx=0;
	uint32_t color;

	color = ((LED_Data[0]<<16) | (LED_Data[1]<<8) | (LED_Data[2]));

	for (int i=23; i>=0; i--)
	{
		if (color&(1<<i)) 
			pwmData_RGB[indx] = 8;  // 14 2/3 of 20

		else
			pwmData_RGB[indx] = 4;  // 6 1/3 of 20

		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)pwmData_RGB, (24 + 50));
	dma_waiting_ws2812 = 1;
}