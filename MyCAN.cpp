#include <STM32_CAN.h>
#include "MyCAN.h"

// Pointer to user-provided STM32_CAN instance
static STM32_CAN* canInstance = nullptr;
static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

extern "C" {

void MyCAN_setInstance(void* CANptr) {
    canInstance = (STM32_CAN*)CANptr;
}

void MyCAN_setConfigurationMode(void* CANptr) {
    (void)CANptr;
    if(canInstance) {
        canInstance->setMode(STM32_CAN::NORMAL);
    }
}

void MyCAN_setNormalMode(void* CANptr) {
    (void)CANptr;
    if(canInstance) {
        canInstance->setMode(STM32_CAN::NORMAL);
    }
}

void MyCAN_configureAndBeginCan(uint16_t CANbitRate) {
    if(canInstance) {
        canInstance->begin();
        canInstance->setBaudRate(CANbitRate);
    }
}

void MyCAN_CANmodule_disable() {
    if(canInstance) {
        canInstance->end();
    }
}

bool MyCAN_write(uint32_t id, const uint8_t* data, uint8_t len) {
    // Check for null data pointer
    if(data == nullptr) {
        return false;
    }
    
    // Check for invalid length (CAN frame can have max 8 bytes of data)
    if(len == 0 || len > 8) {
        return false;
    }
    
    CAN_TX_msg.id = id;
    CAN_TX_msg.flags.extended = 0;
    CAN_TX_msg.len = len;
    
    // Copy data to CAN message buffer
    for(int i = 0; i < len; ++i){
        CAN_TX_msg.buf[i] = data[i];
    }
    
    // Zero out unused bytes in the buffer
    for(int i = len; i < 8; ++i){
        CAN_TX_msg.buf[i] = 0;
    }
    
    // Send the message and check if it was successful
    if(canInstance->write(CAN_TX_msg)) {
        return true;
    } else {
        return false;
    }
}
bool MyCAN_read(My_CAN_message_t *msg) {
    if (!canInstance) return false;
    bool res = canInstance->read(CAN_RX_msg);
    if (!res) return false;
    else {
        msg->id = CAN_RX_msg.id;
        msg->len = CAN_RX_msg.len;
        for (int i = 0; i < CAN_RX_msg.len; ++i) {
            msg->buf[i] = CAN_RX_msg.buf[i];
        }
        msg->flags.extended = CAN_RX_msg.flags.extended;
        msg->flags.remote = CAN_RX_msg.flags.remote;
        //msg->bus = CAN_RX_msg.bus;
    }
}

} // extern "C"
