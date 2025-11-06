#include <STM32_CAN.h>
#include "MyCAN.h"

// Pointer to user-provided STM32_CAN instance
static STM32_CAN* canInstance = nullptr;

extern "C" {

void MyCAN_setInstance(void* CANptr) {
    canInstance = (STM32_CAN*)CANptr;
}

void MyCAN_setConfigurationMode(void* CANptr) {
    (void)CANptr;
    if(canInstance) canInstance->setMode(STM32_CAN::CONFIGURATION);
}

void MyCAN_setNormalMode(void* CANptr) {
    (void)CANptr;
    if(canInstance) canInstance->setMode(STM32_CAN::NORMAL);
}

void MyCAN_write(uint32_t id, uint8_t* data, uint8_t len) {
    if(canInstance) canInstance->send(id, data, len);
}

bool MyCAN_read(uint32_t* id, uint8_t* data, uint8_t* len) {
    if(canInstance) return canInstance->read(id, data, len);
    return false;
}

} // extern "C"
