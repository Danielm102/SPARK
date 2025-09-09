#ifndef AS5600_H_
#define AS5600_H_

#include "main.h"
#include "stdbool.h"

typedef struct {
    bool MH;
    bool ML;
    bool MD;
} AS5600_status_t;

extern int rotation_count;

extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef AS5600_write_reg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef AS5600_read_reg(uint8_t start_reg, uint8_t *data, uint8_t length);

void AS5600_getStatus(AS5600_status_t *status);
void AS5600_readAngleRaw(float *raw_angle);
void AS5600_readAngle(float *angle);

#define AS5600_I2C_ADDR (0x36 << 1)

/* Configuration Registers */
#define AS5600_ZMCO_REG 0x00        // 1 R
#define AS5600_ZPOS_REG 0x01        // 2 RW
#define AS5600_MPOS_REG 0x03        // 2 RW
#define AS5600_MANG_REG 0x05        // 2 RW
#define AS5600_CONF_REG 0x07        // 2 RW

/* Output Registers */
#define AS5600_RAW_ANGLE_REG 0x0C   // 2 R
#define AS5600_ANGLE_REG 0x0E       // 2 R

/* Status Registers */
#define AS5600_STATUS_REG 0x0B      // 1 R
#define AS5600_AGC_REG 0x1A         // 1 R
#define AS5600_MAGNITUDE_REG 0x1C   // 2 R

/* Burn Commands */
#define AS5600_BURN_REG 0xFF        // 1 W
#define AS5600_BURN_ANGLE 0x80
#define AS5600_BURN_SETTING 0x40




#endif /* AS5600_H_ */