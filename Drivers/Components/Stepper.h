#ifndef Stepper_H_
#define Stepper_H_

#include "main.h"

#define MAX_STEPPER_STEPS 1000   // Maximum number of steps to move in one call
#define STEPPER_STEP_TIME 2     // Time in ms for one step

#define CURRENT_SCALE_KV 1.32

#define rod_inclination 8

typedef enum {
    reverse = 0,
    forward = 1
} stepper_dir_t;

extern TIM_HandleTypeDef htim1;

extern SPI_HandleTypeDef hspi1;

extern float pos_Stepper;
extern uint16_t pwmData[MAX_STEPPER_STEPS];

extern volatile uint8_t dma_waiting_stepper;


void Stepper_SetDirection(stepper_dir_t dir);
void Stepper_Init();

uint8_t Stepper_write_reg(uint8_t address, uint8_t data);
uint8_t Stepper_read_reg(uint8_t address, uint8_t *data);

void Stepper_SetTorque(uint8_t torque);
void Stepper_EnableControl();
void Stepper_setMicrostep(uint8_t microstep_mode);

int16_t Stepper_moveSteps(int16_t steps);
void Stepper_movetoPos(float pos_cmd);
void Stepper_Send (int16_t steps);


#define Stepper_Select()    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, 0)
#define Stepper_Deselect()  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, 1)

#define Stepper_Wakeup()    HAL_GPIO_WritePin(DRV_SLEEP_GPIO_Port, DRV_SLEEP_Pin, 1)
#define Stepper_Sleep()     HAL_GPIO_WritePin(DRV_SLEEP_GPIO_Port, DRV_SLEEP_Pin, 0)

#define Stepper_Enable()    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1)
#define Stepper_Disable()   HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0)


#define DRV_SPI_READ 0x40 // W0=1 for read

#define DRV_FAULT_STATUS_REG 0x00   // Type: R
#define DRV_DIAG_STATUS1_REG 0x01   // Type: R
#define DRV_DIAG_STATUS2_REG 0x02   // Type: R
#define DRV_CTRL1_REG 0x03          // Type: RW
#define DRV_CTRL2_REG 0x04          // Type: RW
#define DRV_CTRL3_REG 0x05          // Type: RW
#define DRV_CTRL4_REG 0x06          // Type: RW
#define DRV_CTRL5_REG 0x07          // Type: RW
#define DRV_CTRL6_REG 0x08          // Type: RW
#define DRV_CTRL7_REG 0x09          // Type: RW
#define DRV_CTRL8_REG 0x0A          // Type: R
#define DRV_CTRL9_REG 0x0B          // Type: R

#define DRV_STEP_FULL_100   0b0000  // 100% current
#define DRV_STEP_FULL_71    0b0001  // 71 % current
#define DRV_STEP_1_2_NC     0b0010  // non-circular
#define DRV_STEP_1_2        0b0011
#define DRV_STEP_1_4        0b0100
#define DRV_STEP_1_8        0b0101
#define DRV_STEP_1_16       0b0110
#define DRV_STEP_1_32       0b0111
#define DRV_STEP_1_64       0b1000
#define DRV_STEP_1_128      0b1001
#define DRV_STEP_1_256      0b1010

#define DRV_TRQ_16_16   0b0000
#define DRV_TRQ_15_16   0b0001
#define DRV_TRQ_14_16   0b0010
#define DRV_TRQ_13_16   0b0011
#define DRV_TRQ_12_16   0b0100
#define DRV_TRQ_11_16   0b0101
#define DRV_TRQ_10_16   0b0110
#define DRV_TRQ_09_16   0b0111
#define DRV_TRQ_08_16   0b1000
#define DRV_TRQ_07_16   0b1001
#define DRV_TRQ_06_16   0b1010
#define DRV_TRQ_05_16   0b1011
#define DRV_TRQ_04_16   0b1100
#define DRV_TRQ_03_16   0b1101
#define DRV_TRQ_02_16   0b1110
#define DRV_TRQ_01_16   0b1111

#endif /* Stepper_H_ */