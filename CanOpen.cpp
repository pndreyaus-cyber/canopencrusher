#include <cstring> // for memcpy
#include "CanOpen.h"
#include "RobotConstants.h"

bool CanOpen::send_zeroInitialize(uint8_t nodeId)
{
    /*
    According to the device documentation, 
    zero initialization involves writing two specific values to an Electronic Gear Molecule register (with address 0x260A)  
    
    The first value is 0xEA66 (60006 in decimal)
    The second value is 0xEA70 (60016 in decimal)

    The CAN package use little-endian format, so we need to reverse the byte order when sending
    */
    uint8_t data1[2] = {0x66, 0xEA};
    uint8_t data2[2] = {0x70, 0xEA};

    bool ok = sendSDO(
        nodeId,
        2,
        RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES,
        0x00,
        data1
    );

    if (!ok) {
        Serial2.println("Failed to send first part of zero initialization");
        return false;
    }

    ok = sendSDO(
        nodeId,
        2,
        RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES,
        0x00,
        data2
    );

    if (!ok) {
        Serial2.println("Failed to send second part of zero initialization");
        return false;
    }
    return true;
}

bool CanOpen::send_x260A_electronicGearMolecules(uint8_t nodeId, int32_t value)
{
    return sendSDO(
        nodeId,
        4,
        RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES,
        _ElectronicGearMolecules_ElectronicGearMolecules_sIdx,
        &value
    );
}

bool CanOpen::send_x60FF_targetVelocity(uint8_t nodeId, int32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _Target_velocity_Idx,
        _Target_velocity_Target_velocity_sIdx,
        &value
    );
}

bool CanOpen::send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _Profile_acceleration_Idx,
        _Profile_acceleration_Profile_acceleration_sIdx,
        &value
    );
}

bool CanOpen::send_x6081_profileVelocity(uint8_t nodeId, uint32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _Profile_velocity_Idx,
        _Profile_velocity_Profile_velocity_sIdx,
        &value
    );
}

bool CanOpen::send_x6040_controlword(uint8_t nodeId, uint16_t value)
{
    return sendSDO(
        nodeId,
        2,
        _Controlword_Idx,
        _Controlword_Controlword_sIdx,
        &value
    );
}

bool CanOpen::send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value)
{
    return sendSDO(
        nodeId,
        1,
        _Modes_of_operation_Idx,
        _Modes_of_operation_Modes_of_operation_sIdx,
        &value
    );    
}

bool CanOpen::sendSDO(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void *data)
{
    uint8_t msgBuf[8] = {0};
    
    // SDO expedited write (1-4 bytes, command specifier по размеру данных)
    uint8_t cs;
    switch (dataLen) {
        case 1: cs = 0x2F; break;
        case 2: cs = 0x2B; break;
        case 3: cs = 0x27; break;
        case 4: cs = 0x23; break;
        default: return false; // error
    }
    msgBuf[0] = cs;
    memcpy(&msgBuf[1], &index, 2);
    msgBuf[3] = subindex;
    memcpy(&msgBuf[4], data, dataLen);

    // Отправляем (если ваша библиотека использует CAN.write или queue)
    return send(0x600 + nodeId, msgBuf, dataLen + 4);
}

bool CanOpen::sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute)
{
    uint8_t msgBuf[4] = {0};
    memcpy(msgBuf, &targetPositionAbsolute, 4);
    return send(0x500 + nodeId, msgBuf, 4);
}

bool CanOpen::sendSYNC()
{
    Serial2.println("Sending SYNC");
    return send(0x80, 0, 0);
}

void CanOpen::writeReversedToBuf(const void* data, size_t size, uint8_t* bufStart) {
    const uint8_t* src_bytes = static_cast<const uint8_t*>(data);
    
    for (size_t i = 0; i < size; ++i) {
        bufStart[i] = src_bytes[size - 1 - i];  // Reverse copy
    }
}


bool CanOpen::startCan(uint32_t baudRate)
{
    if(!can_initialized){
        this->canBaudRate = baudRate;
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
        Can.setBaudRate(canBaudRate);
        can_initialized = true;
        Serial2.println("CAN initialized with baud rate: " + String(canBaudRate));
        return true;
    }
    return false; // already initialized
}

bool CanOpen::loopbackTest(){
    Can.enableLoopBack(true);
    Can.begin();
    Can.setBaudRate(canBaudRate);

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


bool CanOpen::send(uint32_t id, const uint8_t *data, uint8_t len)
{
    // Check for null data pointer
    if(data == nullptr && len > 0) {
        Serial2.println("Error: Null data pointer in CAN send");
        return false;
    }
    
    // Check for invalid length (CAN frame can have max 8 bytes of data)
    if(len > 8) {
        Serial2.println("Error: Invalid data length in CAN send");
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

bool CanOpen::receive(uint32_t &id, uint8_t *data, uint8_t &len)
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
