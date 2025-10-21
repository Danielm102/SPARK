#include "AS5600.h"

HAL_StatusTypeDef AS6500_I2C_status;

HAL_StatusTypeDef AS5600_write_reg(uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status;
    uint8_t buffer[2] = {reg, data};

    status = HAL_I2C_Master_Transmit(&hi2c2, AS5600_I2C_ADDR, buffer, 2, HAL_MAX_DELAY);
    return status;
}

HAL_StatusTypeDef AS5600_read_reg(uint8_t start_reg, uint8_t *data, uint8_t length) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c2, AS5600_I2C_ADDR, &start_reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(&hi2c2, AS5600_I2C_ADDR, data, length, HAL_MAX_DELAY);
    return status;
}

void AS5600_getStatus(AS5600_status_t *status) {
    uint8_t data;
    AS5600_read_reg(AS5600_STATUS_REG, &data, 1);
    status->MH = (data & 0x08) >> 3;
    status->ML = (data & 0x10) >> 4;
    status->MD = (data & 0x20) >> 5;
}

void AS5600_readAngleRaw(float *raw_angle) {
    uint8_t data[2];
    uint16_t data_fused = 0;
    AS5600_read_reg(AS5600_RAW_ANGLE_REG, data, 2);
    data[0] &= 0x0F;
    data_fused = (uint16_t)data[1];
    data_fused |= (uint16_t)data[0] << 8;
    *raw_angle = data_fused / -4096.f * 360.f;
}

void AS5600_readAngle(float *angle) {
    uint8_t data[2];
    uint16_t data_fused = 0;
    AS5600_read_reg(AS5600_ANGLE_REG, data, 2);
    data[0] &= 0x0F;
    data_fused = (uint16_t)data[1];
    data_fused |= (uint16_t)data[0] << 8;
    *angle = data_fused / -4096.f * 360.f;
}

void AngleConstrainedToContinuous(float constrained_angle, float *continuous_angle, float *rotation_count) {
    float angle_new = constrained_angle + *rotation_count * 360.f;
    if(angle_new - *continuous_angle < 180) {
        rotation_count++;
        *continuous_angle = angle_new + 360.f;
    } else if(angle_new - *continuous_angle > 180) {
        rotation_count--;
        *continuous_angle = angle_new - 360.f;
    }
}