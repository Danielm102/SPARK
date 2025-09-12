#include "Stepper.h"

HAL_StatusTypeDef DRV_HAL_SPI_status;

DRV8434S_status_t DRV_status;
DRV8434S_diag1_t DRV_diag1;
DRV8434S_diag2_t DRV_diag2;

uint8_t DRV_status_byte;    // holds SPI response status bits. 0xC0 = no faults
float speed_setpoint;
float pos_deviation;

stepper_movement_t stepper;

/* --------------------------------- DRV8434S SPI functions --------------------------------- */

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

void Stepper_getFullStatus() {
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

// default: DRV_OL_RELEASE_AFTER_CLEAR
void Stepper_setOpenLoadMode(bool mode) {
    uint8_t data;
    if(Stepper_read_reg(DRV_CTRL1_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xFD;
    data |= mode << 1;
    if(Stepper_write_reg(DRV_CTRL1_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_TRQ_16_16
void Stepper_setTorque(uint8_t torque) {
    uint8_t data;
    if(Stepper_read_reg(DRV_CTRL1_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x0F;
    data |= torque << 4;
    if(Stepper_write_reg(DRV_CTRL1_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_DECAY_SMART_TUNE_RIPPLE_CONTROL
void Stepper_setDecay(uint8_t decay) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL2_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xF8;
    data |= decay;
    if(Stepper_write_reg(DRV_CTRL2_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// automatically adjusted for smart tune ripple control | default: DRV_TOFF_16_US
void Stepper_setTOFF(uint8_t t_off) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL2_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xE7;
    data |= t_off << 3;
    if(Stepper_write_reg(DRV_CTRL2_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_enableControl() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL2_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data |= 0x80;
    if(Stepper_write_reg(DRV_CTRL2_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_disableControl() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL2_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x7F;
    if(Stepper_write_reg(DRV_CTRL2_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_STEP_FULL_100
void Stepper_setMicrostep(uint8_t microstep_mode) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL3_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xF0;
    data |= microstep_mode;
    if(Stepper_write_reg(DRV_CTRL3_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_INPUT_MODE_PIN
void Stepper_configInputMode(uint8_t input_mode) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL3_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xCF;
    data |= input_mode << 4;
    if(Stepper_write_reg(DRV_CTRL3_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: 0 (reverse)
void Stepper_setDIR_SPI(stepper_dir_t dir) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL3_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x7F;
    data |= dir << 7;
    if(Stepper_write_reg(DRV_CTRL3_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_setSTEP_SPI(bool step) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL3_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xBF;
    data |= step << 6;
    if(Stepper_write_reg(DRV_CTRL3_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_OTW_NO_REPORT_NFAULT, DRV_OTS_MODE_LATCHED_FAULT
void Stepper_setTemperatureFault(bool OTW_report_nFAULT, bool OTS_auto_recovery) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL4_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xFC;
    data |= OTW_report_nFAULT | (OTS_auto_recovery << 1);
    if(Stepper_write_reg(DRV_CTRL4_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_OCP_MODE_LATCHED_FAULT
void Stepper_setOvercurrentFault(bool OC_auto_retry) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL4_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xFB;
    data |= OC_auto_retry << 2;
    if(Stepper_write_reg(DRV_CTRL4_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DISABLE
void Stepper_OpenLoadDetection(bool OL_en) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL4_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xF7;
    data |= OL_en << 3;
    if(Stepper_write_reg(DRV_CTRL4_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_lockRegisters() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL4_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x8F;
    data |= DRV_LOCK_REGISTERS << 4;
    if(Stepper_write_reg(DRV_CTRL4_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_unlockRegisters() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL4_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x8F;
    data |= DRV_UNLOCK_REGISTERS << 4;
    if(Stepper_write_reg(DRV_CTRL4_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_clearFaults() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL4_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data |= 0x80;
    if(Stepper_write_reg(DRV_CTRL4_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_STALL_DETECTION_OFF, DRV_STALL_REPORT_ON_NFAULT
void Stepper_setStallDetection(bool STL_en, bool STL_report) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL5_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xE7;
    data |= (STL_report << 3) | (STL_en << 4);
    if(Stepper_write_reg(DRV_CTRL5_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_learnStallCount() {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL5_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data |= 0x20;
    if(Stepper_write_reg(DRV_CTRL5_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_getStallThreshold(uint16_t *count) {
    uint8_t data = 0;
    uint16_t out;
    Stepper_read_reg(DRV_CTRL6_REG, &data);
    out = (uint16_t)data;

    Stepper_read_reg(DRV_CTRL7_REG, &data);
    data &= 0x0F;
    out |= ((uint16_t)data) << 8;

    *count = out;
}

// 0 - 4095 | default: 3
void Stepper_setStallThreshold(uint16_t count) {
    uint8_t data = 0;
    data = (uint8_t)count;
    if(Stepper_write_reg(DRV_CTRL6_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();

    if(count > 255) {
        if(Stepper_read_reg(DRV_CTRL7_REG, &data) != DRV_STATUS_BYTE_OK) 
            Stepper_FaultHandler();

        data &= 0xF0;
        data |= (uint8_t)(count >> 8);
        if(Stepper_write_reg(DRV_CTRL7_REG, data) != DRV_STATUS_BYTE_OK)
            Stepper_FaultHandler();
    }
}

// default: DRV_TRQ_SCALE_NONE
void Stepper_scaleTorqueCount(bool TRQ_scale) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL7_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xEF;
    data |= TRQ_scale << 4;
    if(Stepper_write_reg(DRV_CTRL7_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: ENABLE
void Stepper_setSpreadSpectrum(bool SSC_en) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL7_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0xDF;
    data |= SSC_en << 5;
    if(Stepper_write_reg(DRV_CTRL7_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

// default: DRV_RC_RIPPLE_1_PERCENT
void Stepper_setRCRipple(uint8_t ripple) {
    uint8_t data = 0;
    if(Stepper_read_reg(DRV_CTRL7_REG, &data) != DRV_STATUS_BYTE_OK) 
        Stepper_FaultHandler();

    data &= 0x3F;
    data |= ripple << 6;
    if(Stepper_write_reg(DRV_CTRL7_REG, data) != DRV_STATUS_BYTE_OK)
        Stepper_FaultHandler();
}

void Stepper_getTRQCount(uint16_t *count) {
    uint8_t data = 0;
    uint16_t out;
    Stepper_read_reg(DRV_CTRL8_REG, &data);
    out = (uint16_t)data;

    Stepper_read_reg(DRV_CTRL9_REG, &data);
    data &= 0x0F;
    out |= ((uint16_t)data) << 8;

    *count = out;
}

void Stepper_getREV_ID(uint8_t *id) {
    uint8_t data = 0;
    Stepper_read_reg(DRV_CTRL9_REG, &data);
    data = data >> 4;
    *id = data;
}


/* ----------------------------- Stepper Motor Control Functions ---------------------------- */

void Stepper_Init() {
    Stepper_Wakeup();

    stepper.mode = target_pos;

    HAL_Delay(5);

    Stepper_setOpenLoadMode(DRV_OL_RELEASE_AFTER_CLEAR);
    Stepper_setTorque(DRV_TRQ_08_16);
    Stepper_setDecay(DRV_DECAY_SMART_TUNE_RIPPLE_CONTROL);
    Stepper_setTOFF(DRV_TOFF_16_US);
    Stepper_setMicrostep(DRV_STEP_SIZE);
    Stepper_configInputMode(DRV_INPUT_MODE_PIN);
    Stepper_setTemperatureFault(DRV_OTW_NO_REPORT_NFAULT, DRV_OTS_MODE_LATCHED_FAULT);
    Stepper_setOvercurrentFault(DRV_OCP_MODE_LATCHED_FAULT);
    Stepper_OpenLoadDetection(DISABLE);
    Stepper_setStallDetection(DRV_STALL_DETECTION_ON, DRV_STALL_REPORT_ON_NFAULT);
    Stepper_setStallThreshold(180);
    Stepper_scaleTorqueCount(DRV_TRQ_SCALE_MLT8);
    Stepper_setSpreadSpectrum(ENABLE);
    Stepper_setRCRipple(DRV_RC_RIPPLE_1_PERCENT);

    Stepper_enableControl();
}

void Stepper_setDirection(stepper_dir_t dir) {
    HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin, dir);
}

/*void Stepper_movetoPos(float pos_cmd) {
    int steps = 200. * (pos_Stepper - pos_cmd) / rod_inclination; // calculate step count
    Stepper_moveSteps(steps); // move
    pos_Stepper = pos_cmd; // update position
}*/

void Stepper_FaultHandler() {
    Stepper_getFullStatus();
}

// update timer settings based on speed commands
void Stepper_setSpeed(float revolutions_per_second) {
    stepper.speed_prev = revolutions_per_second;
    Stepper_Enable();   // make sure stepper is enabled

    // set direction based on sign, stop timer if commanded speed is too low
    if((revolutions_per_second >= -STEPPER_MIN_SPEED) && (revolutions_per_second <= STEPPER_MIN_SPEED)) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return;
    } else if(revolutions_per_second < 0) {
        Stepper_setDirection(reverse);
        revolutions_per_second = -revolutions_per_second;
    } else if(revolutions_per_second > 0) {
        Stepper_setDirection(forward);
    }

    // calculate prescaler for period = 10
    int prescaler = round((float)TIM1_BASE_FREQ/revolutions_per_second/STEPPER_STEPS_PER_REVOLUTION/DRV_STEP_DIV/TIM1_ARR);

    // account for 16 bit prescaler limit by adjusting period
    if(prescaler > 65535) {
        int period = round(prescaler/65536.f + 0.5) * TIM1_ARR;
        __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);

        prescaler = round((float)TIM1_BASE_FREQ/revolutions_per_second/STEPPER_STEPS_PER_REVOLUTION/DRV_STEP_DIV/period);
    } else if(__HAL_TIM_GET_AUTORELOAD(&htim1) != 9) {
        __HAL_TIM_SET_AUTORELOAD(&htim1, TIM1_ARR - 1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM1_ARR / 2);
    }
    
    __HAL_TIM_SET_PRESCALER(&htim1, prescaler - 1);
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

// enter target position mode and set target position
void Stepper_setTargetDeg(float degrees) {
    stepper.mode = target_pos;
    stepper.pos_cmd = degrees + stepper.neutral_angle;
}

// enter target speed mode and set target speed
void Stepper_setTargetSpeed(float deg_s) {
    stepper.mode = target_speed;
    stepper.speed_target = deg_s;
}

// stop stepper without changing mode
void Stepper_stopMoving() {
    Stepper_setSpeed(0);
    stepper.speed_target = 0;
    stepper.pos_cmd = mag_angle;
}

// move by a certain number of degrees
void Stepper_moveDeg(float degrees) {
    if(stepper.mode != target_pos) {
        stepper.mode = target_pos;
        stepper.pos_cmd = mag_angle;
    }
    stepper.pos_cmd += degrees;
}

// update speed based on call frequency and current measured position
void Stepper_updateSpeed(float freq, float pos) {
    stepper.pos_prev = pos;

    switch(stepper.mode) {
        case target_pos:
            // calculate deviation from setpoint
            pos_deviation = pos - stepper.pos_cmd;

            // if position is within tolerance, disable timer
            if(pos_deviation < STEPPER_MAX_POSITION_ERROR && pos_deviation > -STEPPER_MAX_POSITION_ERROR)
                stepper.speed_target = 0;

            else {
                // apply K factor
                pos_deviation *= -0.02;

                // limit stepper angular rate
                stepper.speed_target = fconstrain(pos_deviation, -STEPPER_MAX_SPEED, STEPPER_MAX_SPEED);
            }

        case target_speed:
            // limit stepper angular acceleration
            stepper.speed_cmd = fconstrain((stepper.speed_target - stepper.speed_prev), -STEPPER_MAX_ACCELERATION / freq, STEPPER_MAX_ACCELERATION / freq) + stepper.speed_prev;
    }

    Stepper_setSpeed(stepper.speed_cmd);
}