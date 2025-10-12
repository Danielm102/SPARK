#include "communication.h"

extern SPI_HandleTypeDef hspi2;
extern uint8_t spi2_tx_buffer[32];

DataPacket_t command_packet;
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
    // Process power unit command
  }
}