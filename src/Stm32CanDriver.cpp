#include <Arduino.h>
#include "../include/Stm32CanDriver.h"
#include "../include/Params.h"

Stm32CanDriver::~Stm32CanDriver()
{
}

Stm32CanDriver::Stm32CanDriver(uint32_t baudRate) : Can( CAN1, ALT ) 
{
    Can.setAutoRetransmission(true);
    Can.begin();
    Can.setBaudRate(baudRate);
}

bool Stm32CanDriver::send(uint32_t id, const uint8_t *data, uint8_t len)
{
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
    if(Can.write(CAN_TX_msg)) {
        return true;
    } else {
        return false;
    }
}

bool Stm32CanDriver::receive(uint32_t &id, uint8_t *data, uint8_t &len)
{
    if(Can.read(CAN_RX_msg)) {
        id = CAN_RX_msg.id;
        len = CAN_RX_msg.len;
        for(int i = 0; i < CAN_RX_msg.len; ++i){
            data[i] = CAN_RX_msg.buf[i];
        }
        return true;
    }
    return false;
}
