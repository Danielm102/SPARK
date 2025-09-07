#include "Stepper.h"

HAL_StatusTypeDef DRV_HAL_SPI_status;

DRV8434S_status_t DRV_status;
DRV8434S_diag1_t DRV_diag1;
DRV8434S_diag2_t DRV_diag2;

uint8_t DRV_status_byte;

float pos_Stepper = 0;
uint16_t pwmData[MAX_STEPPER_STEPS];


uint8_t Stepper_write_reg(uint8_t address, uint8_t data) {
    Stepper_Select();

    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};
    tx[0] = (address << 1);
    tx[1] = data;

    // STATUS = 1 | 1 | UVLO | CPUV | OCP | STL | TF | OL
    DRV_HAL_SPI_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    DRV_status_byte = rx[0];

    Stepper_Deselect();
    return rx[0];
}

uint8_t Stepper_read_reg(uint8_t address, uint8_t *data) {
    Stepper_Select();

    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};
    tx[0] = (address << 1) | DRV_SPI_READ;

    // STATUS = 1 | 1 | UVLO | CPUV | OCP | STL | TF | OL
    DRV_HAL_SPI_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    *data = rx[1];
    DRV_status_byte = rx[0];

    Stepper_Deselect();
    return rx[0];
}

void Stepper_GetFullStatus() {
    uint8_t data = 0;
    Stepper_read_reg(DRV_FAULT_STATUS_REG, &data);
    DRV_status.OL        = 0x01 & (data);
    DRV_status.TF        = 0x01 & (data >> 1);
    DRV_status.STL       = 0x01 & (data >> 2);
    DRV_status.OCP       = 0x01 & (data >> 3);
    DRV_status.CPUV      = 0x01 & (data >> 4);
    DRV_status.UVLO      = 0x01 & (data >> 5);
    DRV_status.SPI_ERROR = 0x01 & (data >> 6);
    DRV_status.FAULT     = 0x01 & (data >> 7);

    Stepper_read_reg(DRV_DIAG_STATUS1_REG, &data);
    DRV_diag1.OCP_HS1_A = 0x01 & (data);
    DRV_diag1.OCP_LS1_A = 0x01 & (data >> 1);
    DRV_diag1.OCP_HS2_A = 0x01 & (data >> 2);
    DRV_diag1.OCP_LS2_A = 0x01 & (data >> 3);
    DRV_diag1.OCP_HS1_B = 0x01 & (data >> 4);
    DRV_diag1.OCP_LS1_B = 0x01 & (data >> 5);
    DRV_diag1.OCP_HS2_B = 0x01 & (data >> 6);
    DRV_diag1.OCP_LS2_B = 0x01 & (data >> 7);

    Stepper_read_reg(DRV_DIAG_STATUS2_REG, &data);
    DRV_diag2.OL_A       = 0x01 & (data);
    DRV_diag2.OL_B       = 0x01 & (data >> 1);
    DRV_diag2.STALL      = 0x01 & (data >> 3);
    DRV_diag2.STL_LRN_OK = 0x01 & (data >> 4);
    DRV_diag2.OTS        = 0x01 & (data >> 5);
    DRV_diag2.OTW        = 0x01 & (data >> 6);
}

void Stepper_SetTorque(uint8_t torque) {
    uint8_t data;
    if(Stepper_read_reg(DRV_CTRL1_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x0F;
    data |= torque << 4;
    if(Stepper_write_reg(DRV_CTRL1_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_EnableControl() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL2_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data |= 0x80;
    if(Stepper_write_reg(DRV_CTRL2_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_setMicrostep(uint8_t microstep_mode) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL3_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xF0;
    data |= microstep_mode;
    if(Stepper_write_reg(DRV_CTRL3_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_SetDirection(stepper_dir_t dir) {
    HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin, dir);
}


void Stepper_Init() {
    for (int i = 0; i < MAX_STEPPER_STEPS; i++) {
        pwmData[i] = 8; // Initialize the PWM data array
    }
}

int16_t Stepper_moveSteps(int16_t steps) {
    if (dma_waiting_stepper)
        return steps;   // If DMA is already waiting, do not start a new transfer

    if (steps == 0)
        return 0;       // No steps to move

    Stepper_Enable();   // make sure stepper is enabled
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

void Stepper_FaultHandler() {

}