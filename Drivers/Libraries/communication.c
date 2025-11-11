#include "communication.h"

uint8_t spi2_tx_buffer[32] = {0};

DataPacket_t command_packet;
DataPacket_t spark_data_packet;

uint8_t getCRC(DataPacket_t *packet) {
    // Simple XOR-based CRC calculation
    uint8_t *data = (uint8_t *)packet;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(DataPacket_t) - 1; i++) {
        crc ^= data[i];
    }
    return crc;
}

void UpdateSPARKDataPacket(DataPacket_t *spark_packet, float magAngle, float posDeviation, float voltage_driver, float temperature_driver, float temperature_converter, uint8_t sparkStatus) {
    spark_packet->Packet_ID = PACKET_ID_SPARK;
    spark_packet->timestamp = HAL_GetTick();

    spark_packet->Data.spark.magAngle = magAngle;
    spark_packet->Data.spark.posDeviation = posDeviation;
    spark_packet->Data.spark.voltage_driver = voltage_driver;
    spark_packet->Data.spark.temperature_driver = temperature_driver;
    spark_packet->Data.spark.temperature_converter = temperature_converter;
    spark_packet->Data.spark.sparkStatus = sparkStatus;

    // Prepare SPI transmit buffer
    memcpy(spi2_tx_buffer, spark_packet, sizeof(DataPacket_t));
    spi2_tx_buffer[0] = PACKET_ID_SPARK;
}

void Communication_ActivateReceive(void) {
    if (hspi2.Instance->SR & (SPI_SR_OVR | SPI_SR_RXNE)) {
        // SPI has errors - clear them before reactivating
        __HAL_SPI_CLEAR_OVRFLAG(&hspi2);

        volatile uint32_t dummy;
        while (hspi2.Instance->SR & SPI_SR_RXNE) {
            dummy = hspi2.Instance->DR;  // Read and discard
        }
    }
    HAL_SPI_TransmitReceive_IT(&hspi2, spi2_tx_buffer, (uint8_t*)&command_packet, sizeof(command_packet));
}

void ProcessCommandPacket() {
    // Check the command target

    if (command_packet.Packet_ID == PACKET_ID_COMMAND && command_packet.Data.command.command_target == COMMAND_TARGET_SPARK) {
        // Process SPARK command
        if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_READ_DATA) {
            // No action needed, data will be read in the next SPI transaction
            return;
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_SET_ANGLE) {
            uint8_t *angle_address = command_packet.Data.command.params;
            float angle_target;
            memcpy(&angle_target, angle_address, sizeof(float));
            Stepper_setTargetDeg(angle_target);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_SET_SPEED) {
            uint8_t *speed_address = command_packet.Data.command.params;
            float speed_target;
            memcpy(&speed_target, speed_address, sizeof(float));
            Stepper_setTargetSpeed(speed_target);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_EXIT_MODE) {
            StateMachine_Dispatch(&spark_sm, EVENT_CMD_EXIT_MODE);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_ZERO_STEPPER) {
            StateMachine_Dispatch(&spark_sm, EVENT_CMD_ZERO_STEPPER);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_FIND_MAX) {
            StateMachine_Dispatch(&spark_sm, EVENT_CMD_FIND_MAX);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_MODE_TARGET_POSITION) {
            StateMachine_Dispatch(&spark_sm, EVENT_CMD_TARGET_POSITION);
            uint8_t torque = command_packet.Data.command.params[0];
            if (torque > 16) torque = 16;
            torque = 16 - torque;
            Stepper_setTorque(torque);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_MODE_TARGET_SPEED) {
            StateMachine_Dispatch(&spark_sm, EVENT_CMD_TARGET_SPEED);
            uint8_t torque = command_packet.Data.command.params[0];
            if (torque > 16) torque = 16;
            torque = 16 - torque;
            Stepper_setTorque(torque);
        } else if (command_packet.Data.command.command_id == COMMAND_ID_SPARK_RESET) {
            HAL_NVIC_SystemReset();
        }
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
}