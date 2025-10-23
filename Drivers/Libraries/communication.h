#ifndef COMM_H
#define COMM_H

#include "main.h"

//Compressed version of status.h and InterBoardCom.h from secondary board firmware, only what is needed for SPARK
//Data is received as commands in data packets with ID COMMAND_TARGET_SPARK, no InterBoardPacket_t struct is used

#pragma pack(push, 1)

typedef enum __attribute__((packed)){
    COMMAND_TARGET_NONE = 0x00,
    COMMAND_TARGET_SPARK = 0x04,
    COMMAND_TARGET_ACK = 0x10,
} CommandTarget_t;

typedef enum __attribute__((packed)){
    PACKET_ID_STATUS = 0x01, // VR data packet
    PACKET_ID_POWER = 0x02, // Power data packet

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
} CommandID_t;

typedef struct {
    CommandTarget_t command_target;
    uint8_t command_id;
    uint8_t params[24];
} CommandPayload_t;

typedef union {
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

void ProcessCommandPacket(void);
void Communication_ActivateReceive(void);

#endif /* COMM_H */