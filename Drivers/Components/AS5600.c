#include "AS5600.h"

HAL_StatusTypeDef AS5600_I2C_status;

HAL_StatusTypeDef AS5600_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};

    AS5600_I2C_status = HAL_I2C_Master_Transmit(&hi2c2, AS5600_I2C_ADDR, buffer, 2, HAL_MAX_DELAY);
    return AS5600_I2C_status;
}

HAL_StatusTypeDef AS5600_read_reg(uint8_t start_reg, uint8_t *data, uint8_t length) {
    AS5600_I2C_status = HAL_I2C_Master_Transmit(&hi2c2, AS5600_I2C_ADDR, &start_reg, 1, HAL_MAX_DELAY);
    if (AS5600_I2C_status != HAL_OK) return AS5600_I2C_status;

    AS5600_I2C_status = HAL_I2C_Master_Receive(&hi2c2, AS5600_I2C_ADDR, data, length, HAL_MAX_DELAY);
    return AS5600_I2C_status;
}

HAL_StatusTypeDef AS5600_getStatus(AS5600_status_t *status) {
    uint8_t data;
    if (AS5600_read_reg(AS5600_STATUS_REG, &data, 1) != HAL_OK) return AS5600_I2C_status;
    status->MH = (data & 0x08) >> 3;
    status->ML = (data & 0x10) >> 4;
    status->MD = (data & 0x20) >> 5;
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_readAngleRaw(float *raw_angle) {
    uint8_t data[2];
    uint16_t data_fused = 0;
    if (AS5600_read_reg(AS5600_RAW_ANGLE_REG, data, 2) != HAL_OK) return AS5600_I2C_status;
    data[0] &= 0x0F;
    data_fused = (uint16_t)data[1];
    data_fused |= (uint16_t)data[0] << 8;
    *raw_angle = data_fused / -4096.f * 360.f;
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_readAngle(float *angle) {
    uint8_t data[2];
    uint16_t data_fused = 0;
    if (AS5600_read_reg(AS5600_ANGLE_REG, data, 2) != HAL_OK) return AS5600_I2C_status;
    data[0] &= 0x0F;
    data_fused = (uint16_t)data[1];
    data_fused |= (uint16_t)data[0] << 8;
    *angle = data_fused / -4096.f * 360.f;
    return HAL_OK;
}

bool AS5600_SelfTest() {
    AS5600_status_t status;
    if (AS5600_getStatus(&status) != HAL_OK)
        return false;
    return (status.MD && !status.ML);
}

void AngleConstrainedToContinuous(float constrained_angle, float *continuous_angle, float *full_rotations) {
    float angle_new = constrained_angle + *full_rotations * 360.f;
    if ((angle_new - *continuous_angle) < -180) {
        *full_rotations = *full_rotations + 1;
        *continuous_angle = angle_new + 360.f;
    } else if ((angle_new - *continuous_angle) > 180) {
        *full_rotations = *full_rotations - 1;
        *continuous_angle = angle_new - 360.f;
    } else {
        *continuous_angle = angle_new;
    }
}