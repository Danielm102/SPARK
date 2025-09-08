/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "Stepper.h"
#include "stdbool.h"
#include "status.h"
#include "VoltageReader.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RunOnce();
void Loop_1000Hz();
void Loop_100Hz();
void Loop_10Hz();

uint32_t HAL_GetTickUS(void);
void TimeMeasureStart(void);
uint32_t TimeMeasureStop(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FC_CS_Pin GPIO_PIN_9
#define FC_CS_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_14
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOC
#define NTC1_Pin GPIO_PIN_0
#define NTC1_GPIO_Port GPIOA
#define NTC2_Pin GPIO_PIN_1
#define NTC2_GPIO_Port GPIOA
#define VD1_Pin GPIO_PIN_2
#define VD1_GPIO_Port GPIOA
#define DRV_FAULT_Pin GPIO_PIN_3
#define DRV_FAULT_GPIO_Port GPIOA
#define DRV_VREF_Pin GPIO_PIN_4
#define DRV_VREF_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_0
#define DRV_CS_GPIO_Port GPIOB
#define DRV_STEP_Pin GPIO_PIN_8
#define DRV_STEP_GPIO_Port GPIOA
#define DRV_DIR_Pin GPIO_PIN_9
#define DRV_DIR_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_6
#define DRV_EN_GPIO_Port GPIOC
#define DRV_SLEEP_Pin GPIO_PIN_10
#define DRV_SLEEP_GPIO_Port GPIOA
#define RGB_Pin GPIO_PIN_15
#define RGB_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define FC_MISO_Pin GPIO_PIN_6
#define FC_MISO_GPIO_Port GPIOB
#define FC_MOSI_Pin GPIO_PIN_7
#define FC_MOSI_GPIO_Port GPIOB
#define FC_SCK_Pin GPIO_PIN_8
#define FC_SCK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TICK_US_RESET_THRESHOLD 1000000000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
