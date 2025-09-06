#include "Stepper.h"

HAL_StatusTypeDef DRV_SPI_status;

float pos_Stepper = 0;
uint16_t pwmData[MAX_STEPPER_STEPS];


uint8_t Stepper_write_reg(uint8_t address, uint8_t data) {
    Stepper_Select();

    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};
    tx[0] = (address << 1);
    tx[1] = data;

    // STATUS = 1 | 1 | UVLO | CPUV | OCP | STL | TF | OL
    DRV_SPI_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    Stepper_Deselect();
    return rx[0];
}

uint8_t Stepper_read_reg(uint8_t address, uint8_t *data) {
    Stepper_Select();

    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};
    tx[0] = (address << 1) | DRV_SPI_READ;

    // STATUS = 1 | 1 | UVLO | CPUV | OCP | STL | TF | OL
    DRV_SPI_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    *data = rx[1];

    Stepper_Deselect();
    return rx[0];
}

void Stepper_SetTorque(uint8_t torque) {
    uint8_t data = 0;
    Stepper_read_reg(DRV_CTRL1_REG, &data);

    data &= 0x0F;
    data |= torque << 4;
    Stepper_write_reg(DRV_CTRL1_REG, data);
}

void Stepper_EnableControl() {
    uint8_t data = 0;
    Stepper_read_reg(DRV_CTRL2_REG, &data);

    data |= 0x80;
    Stepper_write_reg(DRV_CTRL2_REG, data);
}

void Stepper_setMicrostep(uint8_t microstep_mode) {
    uint8_t data = 0;
    Stepper_read_reg(DRV_CTRL3_REG, &data);

    data &= 0xF0;
    data |= microstep_mode;
    Stepper_write_reg(DRV_CTRL3_REG, data);
}

void Stepper_SetDirection(stepper_dir_t dir) {
    HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin, dir);
}


void Stepper_Init() {
    for (int i = 0; i < MAX_STEPPER_STEPS; i++) {
        pwmData[i] = 10; // Initialize the PWM data array
    }
}

int16_t Stepper_moveSteps(int16_t steps) {
    if (dma_waiting_stepper)
        return steps;   // If DMA is already waiting, do not start a new transfer

    if (steps == 0)
        return 0;       // No steps to move

    int16_t steps_cmd;

    if (steps > MAX_STEPPER_STEPS)
        steps_cmd = MAX_STEPPER_STEPS;  // Limit to maximum steps
    else if (steps < -MAX_STEPPER_STEPS)
        steps_cmd = -MAX_STEPPER_STEPS; // Limit to minimum steps
    
    

    // Set direction based on sign of steps
    if (steps > 0) {
        Stepper_SetDirection(forward);
        steps_cmd = steps;          // Use positive step count
    } else {
        Stepper_SetDirection(reverse);
        steps_cmd = -steps;         // Make step count positive
    }

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, steps_cmd);
    dma_waiting_stepper = 1;

    return steps - steps_cmd; // Return remaining steps
}

void Stepper_movetoPos(float pos_cmd) {
    int steps = 200. * (pos_Stepper - pos_cmd) / rod_inclination; // calculate step count
    Stepper_moveSteps(steps); // move
    pos_Stepper = pos_cmd; // update position
}