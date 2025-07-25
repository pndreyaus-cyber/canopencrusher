#include "Stm32CanDriver.h"
#include "Arduino.h"

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
    CAN_TX_msg.id = id;
    CAN_TX_msg.flags.extended = 0;
    CAN_TX_msg.len = len;
    for(int i = 0; i < len; ++i){
        CAN_TX_msg.buf[i] = data[i];
    }
    for(int i = len; i < 8; ++i){
        CAN_TX_msg.buf[i] = 0;
    }
    return Can.write(CAN_TX_msg);
}

bool Stm32CanDriver::receive(uint32_t &id, uint8_t *data, uint8_t &len)
{
    if(Can.read(CAN_RX_msg)) {
        id = CAN_RX_msg.id;
        len = CAN_RX_msg.len;
        for(int i = 0; i < len; ++i){
            data[i] = CAN_RX_msg.buf[i];
        }
        return true;
    }
    return false;
}
