#include <cstring> // for memcpy
#include "CanOpen.h"
#include "RobotConstants.h"

bool CanOpen::send_zeroInitialize(uint8_t nodeId, int commandNum)
{
    /*
    STATUS: IN CONSTRUCTION. DO NOT USE YET.
    According to the device documentation, 
    zero initialization involves writing two specific values to an Electronic Gear Molecule register (with address 0x260A)  
    
    The first value is 0xEA66 (60006 in decimal)
    The second value is 0xEA70 (60016 in decimal)

    The CAN package uses little-endian format, so we need to reverse the byte order when sending
    */
    uint8_t data1[2] = {0x66, 0xEA};
    uint8_t data2[2] = {0x70, 0xEA};

    if (nodeId > RobotConstants::Robot::MAX_NODE_ID) {
        Serial2.println("Invalid nodeId for zero initialization");
        return false;
    }

    if (commandNum != 1 && commandNum != 2) {
        Serial2.println("Invalid commandNum for zero initialization");
        return false;
    }

    return send_x260A_electronicGearMolecules(nodeId, (commandNum == 1) ? 0xEA66 : 0xEA70);
}

bool CanOpen::send_x260A_electronicGearMolecules(uint8_t nodeId, uint16_t value)
{
    return sendSDOWrite(
        nodeId,
        2,
        RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES,
        _ElectronicGearMolecules_ElectronicGearMolecules_sIdx,
        &value
    );
}

bool CanOpen::send_x60FF_targetVelocity(uint8_t nodeId, int32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::TARGET_VELOCITY,
        _Target_velocity_Target_velocity_sIdx,
        &value
    );
}

bool CanOpen::send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::PROFILE_ACCELERATION,
        _Profile_acceleration_Profile_acceleration_sIdx,
        &value
    );
}

bool CanOpen::send_x6081_profileVelocity(uint8_t nodeId, uint32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::PROFILE_VELOCITY,
        _Profile_velocity_Profile_velocity_sIdx,
        &value
    );
}

bool CanOpen::send_x6040_controlword(uint8_t nodeId, uint16_t value)
{
    return sendSDOWrite(
        nodeId,
        2,
        RobotConstants::ODIndices::CONTROLWORD,
        _Controlword_Controlword_sIdx,
        &value
    );
}

/*
Modes of operation:
- 
*/
bool CanOpen::send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value)
{
    return sendSDOWrite(
        nodeId,
        1,
        RobotConstants::ODIndices::MODES_OF_OPERATION,
        _Modes_of_operation_Modes_of_operation_sIdx,
        &value
    );    
}

bool CanOpen::sendSDOWrite(uint8_t nodeId, uint8_t dataLenBytes, uint16_t index, uint8_t subindex, const void *data)
{
    uint8_t msgBuf[RobotConstants::CANOpen::HEADER_SIZE +
                   RobotConstants::CANOpen::MAX_SDO_WRITE_DATA_SIZE ] = {0}; // 8 bytes of CAN message data
    
    // SDO expedited write (1-4 bytes, command specifier по размеру данных)
    uint8_t cs;
    switch (dataLenBytes) {
        case 1: cs = 0x2F; break;
        case 2: cs = 0x2B; break;
        case 3: cs = 0x27; break;
        case 4: cs = 0x23; break;
        default: return false; // error
    }
    // Set function code
    msgBuf[0] = cs;
    // Change index to little-endian format
    msgBuf[1] = static_cast<uint8_t>(index & 0xFF);
    msgBuf[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    // Set subindex
    msgBuf[3] = subindex;
    // Copy data in reverse order for little-endian format
    memcpy(&msgBuf[4], data, dataLenBytes);

    return send(0x600 + nodeId, msgBuf, dataLenBytes + 4);
}
//Example: Sending SDO request to read position: "40 64 60 00"
bool CanOpen::sendSDORead(uint8_t nodeId, uint16_t index, uint8_t subindex) {
    uint8_t msgBuf[4] = {0};
    msgBuf[0] = 0x40; // SDO read command specifier
    // Change index to little-endian format
    msgBuf[1] = static_cast<uint8_t>(index & 0xFF);
    msgBuf[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    // Set subindex
    msgBuf[3] = subindex;

    return send(
        0x600 + nodeId,
        msgBuf,
        4
    );
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
    return send(0x80, nullptr, 0);
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


bool CanOpen::send(uint32_t id, const uint8_t *msgData, uint8_t msgDataLen) // data contains not only data but also SDO command specifier, index, subindex etc.
{
    // Check for null data pointer
    if(msgData == nullptr && msgDataLen > 0) {
        Serial2.println("Error: Null data pointer in CAN send");
        return false;
    }
    
    // Check for invalid length (CAN frame can have max 8 bytes of data)
    if(msgDataLen > 8) {
        Serial2.println("Error: Invalid data length in CAN send");
        return false;
    }
    
    CAN_TX_msg.id = id;
    CAN_TX_msg.flags.extended = 0;
    CAN_TX_msg.len = 8;
    
    // Copy data to CAN message buffer
    for(int i = 0; i < msgDataLen; ++i){
        CAN_TX_msg.buf[i] = msgData[i];
    }
    
    // Zero out unused bytes in the buffer. Necessary, because the buffer may contain old data
    for(int i = msgDataLen; i < 8; ++i){
        CAN_TX_msg.buf[i] = 0;
    }
    
    return Can.write(CAN_TX_msg);
}

bool CanOpen::receive(uint16_t &cob_id, uint8_t *data, uint8_t &len)
{
    if(Can.read(CAN_RX_msg)) {
        cob_id = CAN_RX_msg.id;
        len = CAN_RX_msg.len;
        for(int i = 0; i < CAN_RX_msg.len; ++i){
            data[i] = CAN_RX_msg.buf[i];
        }
        return true;
    }
    return false;
}

bool CanOpen::read() 
{
    uint16_t id;
    uint8_t data[8];
    uint8_t len;

    if (receive(id, data, len)) {
        uint16_t node = id & 0x7F; // Extract node ID from COB-ID
        uint16_t function_code = id & 0x780; // Extract base COB-ID

        if (function_code == RobotConstants::CANOpen::COB_ID_HEARTBEAT_BASE) {
            String status;
            switch (data[0]) {
                case 0x05:
                    status = "operational";
                    break;
                case 0x04:
                    status = "alarm";
                    break;
                case 0x7F:
                    status = "pre-operational";
                    break;
                case 0x00:
                    status = "boot-up";
                    break;
                default:
                    status = "unknown";
                    break;
            }

            //Serial2.println("Heartbeat " + String(node) + ": " + status);
            return true;
        } else if (function_code == RobotConstants::CANOpen::COB_ID_SDO_CLIENT_BASE) { // SDO READ/WRITE RESPONSE
            Serial2.println("SDO Response from node " + String(node));
            
            // Accept 4-byte write acks and 8-byte read responses
            if (len < 4) {
                Serial2.println("Invalid SDO response length from node " + String(node) + ": " + String(len));
                return false;
            }

            uint16_t registerAddress = data[1] | (data[2] << 8);

            if (data[0] == 0x80) {
                Serial2.println("SDO Error response from node " + String(node));
                if (registerAddress == RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES) {
                    if (electronicGearMoleculesWriteStatusCallback) {
                        electronicGearMoleculesWriteStatusCallback(node, false);
                    }
                }
                return false;
            } else if (data[0] == 0x60) { // Response after successful write
                Serial2.println("SDO Write Acknowledged for register " + String(registerAddress, HEX) + " from node " + String(node));
                if (registerAddress == RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES) {
                    if (electronicGearMoleculesWriteStatusCallback) {
                        electronicGearMoleculesWriteStatusCallback(node, true);
                    }
                }
            } else if ((data[0] & 0xF0) == 0x40) { // Assume it's a read response. Why exactly 0x40? Because expedited read responses have this pattern. Why assume? Cannot we definitiely say, that it is a read respones?

                uint8_t registerSize;
                switch (data[0] & 0xF) {
                    case 3: registerSize = 4; break;
                    case 11: registerSize = 2; break;
                    case 15: registerSize = 1; break;
                    default: registerSize = 0; break; // error
                }

                if (registerAddress == RobotConstants::ODIndices::POSITION_ACTUAL_VALUE) {
                    if (registerSize != 4) {
                        Serial2.println("Unexpected register size for Position Actual Value");
                        return false;
                    }

                    int32_t positionValue = (static_cast<int32_t>(data[7]) << 24) |
                                            (static_cast<int32_t>(data[6]) << 16) |
                                            (static_cast<int32_t>(data[5]) << 8)  |
                                            (static_cast<int32_t>(data[4]));

                    Serial2.print("Position Actual Value from node ");
                    Serial2.print(node);
                    Serial2.print(": ");
                    Serial2.println(positionValue, HEX);
                    if (sdoReadPositionCallback) {
                        sdoReadPositionCallback(node, positionValue);
                    }
                    return true;
                }
            }
            return false;
        }
    }
    return false;   
}