#ifndef COMM_H
#define COMM_H

#include "main.h"

//Compressed version of status.h and InterBoardCom.h from secondary board firmware, only what is needed for SPARK
//Data is received as commands in data packets with ID COMMAND_TARGET_SPARK, no InterBoardPacket_t struct is used

extern SPI_HandleTypeDef hspi2;
extern uint8_t spi2_tx_buffer[32];

#pragma pack(push, 1)

typedef enum __attribute__((packed)){
    COMMAND_TARGET_NONE = 0x00,
    COMMAND_TARGET_SPARK = 0x04,
    COMMAND_TARGET_ACK = 0x10,
} CommandTarget_t;

typedef enum __attribute__((packed)){
    PACKET_ID_STATUS = 0x01, // VR data packet
    PACKET_ID_POWER = 0x02, // Power data packet
    PACKET_ID_SPARK = 0x09, // SPARK data packet
    PACKET_ID_COMMAND = 0x10, // Command packet
} PacketType_t;

typedef enum __attribute__((packed)){
    COMMAND_ID_SPARK_SET_ANGLE = 0x00,
    COMMAND_ID_SPARK_SET_SPEED = 0x01,
    COMMAND_ID_SPARK_EXIT_MODE = 0x02,
    COMMAND_ID_SPARK_ZERO_STEPPER = 0x03,
    COMMAND_ID_SPARK_FIND_MAX = 0x04,
    COMMAND_ID_SPARK_MODE_TARGET_POSITION = 0x05,
    COMMAND_ID_SPARK_MODE_TARGET_SPEED = 0x06,
    COMMAND_ID_SPARK_RESET = 0x07,
    COMMAND_ID_SPARK_READ_DATA = 0x08
} CommandID_t;

typedef struct {
  float magAngle;
  float magSpeed;
  float posDeviation;
  float voltage_driver;
  float temperature_driver;
  float temperature_converter;
  uint8_t sparkStatus;
  uint8_t reserved[1];
} SPARKPayload_t;

typedef struct {
    CommandTarget_t command_target;
    uint8_t command_id;
    uint8_t params[24];
} CommandPayload_t;

typedef union {
    SPARKPayload_t spark;
    CommandPayload_t command;
    uint8_t raw[26];
} PayloadData_u;

typedef struct {
    uint8_t Packet_ID; // Packet ID
    uint32_t timestamp;
    PayloadData_u Data;
    uint8_t crc;
} DataPacket_t;
#pragma pack(pop)

extern DataPacket_t spark_data_packet;

uint8_t getCRC(DataPacket_t *packet);

void UpdateSPARKDataPacket(DataPacket_t *spark_packet, float magAngle, float magSpeed, float posDeviation, float voltage_driver, float temperature_NTC1, float temperature_NTC2, uint8_t sparkStatus);
void ProcessCommandPacket(void);
void Communication_ActivateReceive(void);

#endif /* COMM_H */