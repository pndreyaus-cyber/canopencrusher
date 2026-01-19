#include "Stm32CanDriver.h"
#include "Arduino.h"
#include "Params.h"

Stm32CanDriver::~Stm32CanDriver()
{
}

Stm32CanDriver::Stm32CanDriver(uint32_t baudRate) : Can( CAN1, DEF ) 
{
    Can.setAutoRetransmission(true);
    Can.begin();
    Can.setBaudRate(baudRate);
    
    digitalWrite(PC13, LOW);
    //delay(5000);
}

bool Stm32CanDriver::send(uint32_t id, const uint8_t *data, uint8_t len)
{
    //Serial2.println("Sending usinf Stm32CanDriver.cpp");
    if(id == 0x80){
        Serial2.println("Sending SYNC message stm32candriver");
    }
    // Check for null data pointer
    //if(data == nullptr) {
    //    return false;
    //}
    
    // Check for invalid length (CAN frame can have max 8 bytes of data)
    //if(len == 0 || len > 8) {
    //    return false;
    //}
    
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
        if(id == 0x80){
            Serial2.println("SYNC message sent stm32candriver");
        }
        //Serial2.println("CAN write success");
        return true;
    } else {
        if(id == 0x80){
            Serial2.println("SYNC message send failed stm32candriver");
        }
        //Serial2.println("CAN write fail");
        return false;
    }
}

bool Stm32CanDriver::receive(ReceivedMessage& msg)
{
    if(Can.read(CAN_RX_msg)) {
        msg.id = CAN_RX_msg.id;
        msg.len = CAN_RX_msg.len;
        for(int i = 0; i < msg.len; ++i){
            msg.data[i] = CAN_RX_msg.buf[i];
        }
        return true;
    }
    return false;
}
