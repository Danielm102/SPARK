#ifndef Stepper_H_
#define Stepper_H_

#include "main.h"
#include "stdbool.h"

#define STEPPER_STEP_TIME 500   // Time in us for one step (irrelevant)

#define TIM1_BASE_FREQ  16000000
#define TIM1_ARR        10

#define DRV_STEP_SIZE                   DRV_STEP_1_256
#define DRV_STEP_DIV                    256
#define STEPPER_STEPS_PER_REVOLUTION    200
#define STEPPER_MAX_ACCELERATION        15     // revolutions/s²
#define STEPPER_MAX_SPEED               3      // revolutions/s
#define STEPPER_MIN_SPEED               0.0001 // revolutions/s
#define STEPPER_MAX_POSITION_ERROR      0.05   // revolutions

#define CURRENT_SCALE_KV 1.32

#define rod_inclination

typedef enum {
    reverse = 0,
    forward = 1
} stepper_dir_t;

typedef enum {
    target_pos,
    target_speed
} stepper_mode_t;

typedef struct {
    bool FAULT;
    bool SPI_ERROR;
    bool UVLO;      // supply undervoltage lockout
    bool CPUV;      // charge pump undervoltage
    bool OCP;       // overcurrent condition
    bool STL;       // motor stall condition
    bool TF;        // temperature warning or overtemperature shutdown
    bool OL;        // open load condition
} DRV8434S_status_t;

typedef struct {
    bool OCP_HS1_A; // overcurrent: AOUT, half bridge 1, high-side
    bool OCP_LS1_A; // overcurrent: AOUT, half bridge 1, low-side
    bool OCP_HS2_A; // overcurrent: AOUT, half bridge 2, high-side
    bool OCP_LS2_A; // overcurrent: AOUT, half bridge 2, low-side
    bool OCP_HS1_B; // overcurrent: BOUT, half bridge 1, high-side
    bool OCP_LS1_B; // overcurrent: BOUT, half bridge 1, low-side
    bool OCP_HS2_B; // overcurrent: BOUT, half bridge 2, high-side
    bool OCP_LS2_B; // overcurrent: BOUT, half bridge 2, low-side
} DRV8434S_diag1_t;

typedef struct {
    bool OTW;        // overtemperature warning
    bool OTS;        // overtemperature shutdown
    bool STL_LRN_OK; // stall detection learning success
    bool STALL;      // motor stall condition
    bool OL_B;       // open-load detection on BOUT
    bool OL_A;       // open-load detection on AOUT
} DRV8434S_diag2_t;

typedef struct {
    bool active;
    stepper_mode_t mode;
    float neutral_angle;
    float pos_prev;
    float pos_cmd;
    float speed_prev;
    float speed_cmd;
    float speed_target;
    uint16_t stall_count;
    uint16_t TRQ;
} stepper_movement_t;

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;

extern DRV8434S_status_t DRV_status;
extern DRV8434S_diag1_t DRV_diag1;
extern DRV8434S_diag2_t DRV_diag2;

extern stepper_movement_t stepper;

extern uint8_t DRV_status_byte;

// DRV8434S SPI functions
uint8_t Stepper_write_reg(uint8_t address, uint8_t data);
uint8_t Stepper_read_reg(uint8_t address, uint8_t *data);
HAL_StatusTypeDef Stepper_getFullStatus();
HAL_StatusTypeDef Stepper_setOpenLoadMode(bool mode);
HAL_StatusTypeDef Stepper_setTorque(uint8_t torque);
HAL_StatusTypeDef Stepper_setDecay(uint8_t decay);
HAL_StatusTypeDef Stepper_setTOFF(uint8_t t_off);
HAL_StatusTypeDef Stepper_enableControl();
HAL_StatusTypeDef Stepper_disableControl();
HAL_StatusTypeDef Stepper_setMicrostep(uint8_t microstep_mode);
HAL_StatusTypeDef Stepper_configInputMode(uint8_t input_mode);
HAL_StatusTypeDef Stepper_setDIR_SPI(stepper_dir_t dir);
HAL_StatusTypeDef Stepper_setSTEP_SPI(bool step);
HAL_StatusTypeDef Stepper_setTemperatureFault(bool OTW_report_nFAULT, bool OTS_auto_recovery);
HAL_StatusTypeDef Stepper_setOvercurrentFault(bool OC_auto_retry);
HAL_StatusTypeDef Stepper_OpenLoadDetection(bool OL_en);
HAL_StatusTypeDef Stepper_lockRegisters();
HAL_StatusTypeDef Stepper_unlockRegisters();
HAL_StatusTypeDef Stepper_clearFaults();
HAL_StatusTypeDef Stepper_setStallDetection(bool STL_en, bool STL_report);
HAL_StatusTypeDef Stepper_learnStallCount();
HAL_StatusTypeDef Stepper_getStallThreshold(uint16_t *count);
HAL_StatusTypeDef Stepper_setStallThreshold(uint16_t count);
HAL_StatusTypeDef Stepper_scaleTorqueCount(bool TRQ_scale);
HAL_StatusTypeDef Stepper_setSpreadSpectrum(bool SSC_en);
HAL_StatusTypeDef Stepper_setRCRipple(uint8_t ripple);
HAL_StatusTypeDef Stepper_getTRQCount(uint16_t *count);
HAL_StatusTypeDef Stepper_getREV_ID(uint8_t *id);
bool Stepper_SelfTest();

// Stepper motor control functions
HAL_StatusTypeDef Stepper_Init();
void Stepper_setDirection(stepper_dir_t dir);
void Stepper_moveSteps(int16_t steps);
void Stepper_movetoPos(float pos_cmd);

// Error handling
void Stepper_FaultHandler();

// testing
void Stepper_setSpeed(float revolutions_per_second);
void Stepper_setTargetDeg(float degrees);
void Stepper_setTargetSpeed(float deg_s);
void Stepper_stopMoving();
void Stepper_moveDeg(float degrees);
void Stepper_updateSpeed(float freq, float pos);

#define Stepper_Select()    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, 0)
#define Stepper_Deselect()  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, 1)

#define Stepper_Wakeup()    HAL_GPIO_WritePin(DRV_SLEEP_GPIO_Port, DRV_SLEEP_Pin, 1)
#define Stepper_Sleep()     HAL_GPIO_WritePin(DRV_SLEEP_GPIO_Port, DRV_SLEEP_Pin, 0)

#define Stepper_Enable()    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1)
#define Stepper_Disable()   HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0)


#define DRV_SPI_READ 0x40 // W0=1 for read

#define DRV_STATUS_BYTE_OK 0xC0

// register addresses
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

// CTRL1 bitfields
#define DRV_OL_RELEASE_AFTER_CLEAR 0    // default
#define DRV_OL_RELEASE_IMMEDIATELY 1

#define DRV_TRQ_16_16   0b0000  // 100% | default
#define DRV_TRQ_15_16   0b0001  // 93.75%
#define DRV_TRQ_14_16   0b0010  // 87.5%
#define DRV_TRQ_13_16   0b0011  // 81.25%
#define DRV_TRQ_12_16   0b0100  // 75%
#define DRV_TRQ_11_16   0b0101  // 68.75%
#define DRV_TRQ_10_16   0b0110  // 62.5%
#define DRV_TRQ_09_16   0b0111  // 56.25%
#define DRV_TRQ_08_16   0b1000  // 50%
#define DRV_TRQ_07_16   0b1001  // 43.75%
#define DRV_TRQ_06_16   0b1010  // 37.5%
#define DRV_TRQ_05_16   0b1011  // 31.25%
#define DRV_TRQ_04_16   0b1100  // 25%
#define DRV_TRQ_03_16   0b1101  // 18.75%
#define DRV_TRQ_02_16   0b1110  // 12.5%
#define DRV_TRQ_01_16   0b1111  // 6.25%

// CTRL2 bitfields
#define DRV_DECAY_ISLOW_DSLOW               0b000 // increasing SLOW, decreasing SLOW
#define DRV_DECAY_ISLOW_DMI30               0b001 // increasing SLOW, decreasing MIXED 30%
#define DRV_DECAY_ISLOW_DMI60               0b010 // increasing SLOW, decreasing MIXED 60%
#define DRV_DECAY_ISLOW_DFAST               0b011 // increasing SLOW, decreasing FAST
#define DRV_DECAY_IMI30_DMI30               0b100 // increasing & decreasing MIXED 30%
#define DRV_DECAY_IMI60_DMI60               0b101 // increasing & decreasing MIXED 60%
#define DRV_DECAY_SMART_TUNE_DYNAMIC_DECAY  0b110 // smart tune dynamic decay
#define DRV_DECAY_SMART_TUNE_RIPPLE_CONTROL 0b111 // smart tune ripple control | default

#define DRV_TOFF_07_US 0b00
#define DRV_TOFF_16_US 0b01 // default
#define DRV_TOFF_24_US 0b10
#define DRV_TOFF_32_US 0b11

// CTRL3 bitfields
#define DRV_STEP_FULL_100   0b0000  // 100% current full-step | default
#define DRV_STEP_FULL_71    0b0001  // 71 % current full-step
#define DRV_STEP_1_2_NC     0b0010  // non-circular 1/2 step
#define DRV_STEP_1_2        0b0011  // 1/2 step
#define DRV_STEP_1_4        0b0100  // 1/4 step
#define DRV_STEP_1_8        0b0101  // 1/8 step
#define DRV_STEP_1_16       0b0110  // 1/16 step
#define DRV_STEP_1_32       0b0111  // 1/32 step
#define DRV_STEP_1_64       0b1000  // 1/64 step
#define DRV_STEP_1_128      0b1001  // 1/128 step
#define DRV_STEP_1_256      0b1010  // 1/256 step

#define DRV_INPUT_MODE_PIN  0b00    // outputs follow input pins | default
#define DRV_INPUT_MODE_SPI  0b11    // outputs follow spi registers

// CTRL4 bitfields
#define DRV_OTW_NO_REPORT_NFAULT    0   // default
#define DRV_OTW_REPORT_ON_NFAULT    1
#define DRV_OTS_MODE_LATCHED_FAULT  0   // default
#define DRV_OTS_MODE_AUTO_RETRY     1

#define DRV_OCP_MODE_LATCHED_FAULT  0   // default
#define DRV_OCP_MODE_AUTO_RETRY     1

#define DRV_UNLOCK_REGISTERS    0b011   // default
#define DRV_LOCK_REGISTERS      0b110

// CTRL5 bitfields
#define DRV_STALL_DETECTION_OFF     0   // default
#define DRV_STALL_DETECTION_ON      1
#define DRV_STALL_NO_REPORT_NFAULT  0
#define DRV_STALL_REPORT_ON_NFAULT  1   // default

//CTRL7 bitfields
#define DRV_TRQ_SCALE_NONE  0   // no torque count scaling is applied | default
#define DRV_TRQ_SCALE_MLT8  1   // torque count is scaled up by a factor of 8

#define DRV_RC_RIPPLE_1_PERCENT 0b00    // default
#define DRV_RC_RIPPLE_2_PERCENT 0b01
#define DRV_RC_RIPPLE_4_PERCENT 0b10
#define DRV_RC_RIPPLE_6_PERCENT 0b11

#endif /* Stepper_H_ */