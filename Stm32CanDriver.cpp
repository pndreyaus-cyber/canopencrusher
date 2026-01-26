#include "Stm32CanDriver.h"
#include "Arduino.h"
#include "Params.h"

// Stm32CanDriver::Stm32CanDriver() 
// {
//     Serial2.println("Stm32CanDriver Constructor called");
// }

Stm32CanDriver::~Stm32CanDriver()
{
    Serial2.println("Stm32CanDriver Destructor called");
}

bool Stm32CanDriver::start(uint32_t baudRate)
{
    if(!initialized){
        this->baudRate = baudRate;
        Can.setAutoRetransmission(true);

        // Loopback test
        if(!loopbackTest()){
            Serial2.println("CAN loopback test failed during start");
            return false;
        }
        Can.end();

        // Start CAN in normal mode
        Can.enableLoopBack(false);
        Can.begin();
        Can.setBaudRate(baudRate);
        initialized = true;
        Serial2.println("CAN initialized with baud rate: " + String(baudRate));
        return true;
    }
    return false; // already initialized
}

bool Stm32CanDriver::loopbackTest(){
    Can.enableLoopBack(true);
    Can.begin();
    Can.setBaudRate(baudRate);

    CAN_message_t testMsg;
    testMsg.id = 0x123;
    testMsg.flags.extended = 0;
    testMsg.len = 8;
    testMsg.buf[0] = 0xAA;
    testMsg.buf[1] = 0xBB;
    testMsg.buf[2] = 0xCC;
    testMsg.buf[3] = 0xDD;
    testMsg.buf[4] = 0xEE;
    testMsg.buf[5] = 0xFF;
    testMsg.buf[6] = 0x11;
    testMsg.buf[7] = 0x22;

    bool queued = Can.write(testMsg);
    if (!queued) {
        Serial2.println("Failed to queue test message for transmission");
        return false;
    } else {
        Serial2.println("Test message queued for transmission");
    }

    delay(100); // Wait for message to loop back

    CAN_message_t receivedMsg;
    bool got = false;
    if (Can.read(receivedMsg)) {
        got = true;
    } else {
        Serial2.println("Failed to receive loopback message");
        return false;
    }

    Serial2.println("Received loopback message with ID: " + String(receivedMsg.id, HEX));
    for (int i = 0; i < receivedMsg.len; ++i) {
        Serial2.print(receivedMsg.buf[i], HEX);
        Serial2.print(" ");
    }

    if (got && receivedMsg.id == testMsg.id && receivedMsg.len == testMsg.len) {
        bool dataMatch = true;
        for (int i = 0; i < testMsg.len; ++i) {
            if (receivedMsg.buf[i] != testMsg.buf[i]) {
                dataMatch = false;
                break;
            }
        }
        if (dataMatch) {
            Serial2.println("\nLoopback test successful");
            return true;
        } else {
            Serial2.println("\nData mismatch in loopback test");
            return false;
        }
    } else {
        Serial2.println("\nLoopback test failed: ID or length mismatch");
        return false;
    }
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
