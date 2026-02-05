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

    if (nodeId > RobotConstants::Robot::MAX_NODE_ID)
    {
        addDataToOutQueue("Invalid nodeId for zero initialization");
        return false;
    }

    if (commandNum != 1 && commandNum != 2)
    {
        addDataToOutQueue("Invalid commandNum for zero initialization");
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
        &value);
}

bool CanOpen::send_x60FF_targetVelocity(uint8_t nodeId, int32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::TARGET_VELOCITY,
        _Target_velocity_Target_velocity_sIdx,
        &value);
}

bool CanOpen::send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::PROFILE_ACCELERATION,
        _Profile_acceleration_Profile_acceleration_sIdx,
        &value);
}

bool CanOpen::send_x6081_profileVelocity(uint8_t nodeId, uint32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::PROFILE_VELOCITY,
        _Profile_velocity_Profile_velocity_sIdx,
        &value);
}

bool CanOpen::send_x6040_controlword(uint8_t nodeId, uint16_t value)
{
    return sendSDOWrite(
        nodeId,
        2,
        RobotConstants::ODIndices::CONTROLWORD,
        _Controlword_Controlword_sIdx,
        &value);
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
        &value);
}

bool CanOpen::send_x607A_targetPosition(uint8_t nodeId, int32_t value)
{
    return sendSDOWrite(
        nodeId,
        4,
        RobotConstants::ODIndices::TARGET_POSITION,
        _Target_position_Target_position_sIdx,
        &value);
}

bool CanOpen::sendSDOWrite(uint8_t nodeId, uint8_t dataLenBytes, uint16_t index, uint8_t subindex, const void *data)
{
    uint8_t msgBuf[RobotConstants::CANOpen::HEADER_SIZE +
                   RobotConstants::CANOpen::MAX_SDO_WRITE_DATA_SIZE] = {0}; // 8 bytes of CAN message data

    // SDO expedited write (1-4 bytes, command specifier по размеру данных)
    uint8_t cs;
    switch (dataLenBytes)
    {
    case 1:
        cs = 0x2F;
        break;
    case 2:
        cs = 0x23; // CHANGE THAT!!!
        break;
    case 3:
        cs = 0x27;
        break;
    case 4:
        cs = 0x23;
        break;
    default:
        return false; // error
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
// Example: Sending SDO request to read position: "40 64 60 00"
bool CanOpen::sendSDORead(uint8_t nodeId, uint16_t index, uint8_t subindex)
{
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
        4);
}

bool CanOpen::sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute)
{
    uint8_t msgBuf[4] = {0};
    memcpy(msgBuf, &targetPositionAbsolute, 4);
    return send(0x500 + nodeId, msgBuf, 4);
}

bool CanOpen::sendSYNC()
{
    addDataToOutQueue("Sending SYNC");
    return send(0x80, nullptr, 0);
}

bool CanOpen::startCan(uint32_t baudRate)
{
    if (!can_initialized)
    {
        this->canBaudRate = baudRate;
        Can.setAutoRetransmission(true);

        // Loopback test
        if (!loopbackTest())
        {
            addDataToOutQueue("CAN loopback test failed during start");
            return false;
        }
        Can.end();

        // Start CAN in normal mode
        Can.enableLoopBack(false);
        Can.begin();
        Can.setBaudRate(canBaudRate);
        can_initialized = true;
        addDataToOutQueue("CAN initialized with baud rate: " + String(canBaudRate));
        return true;
    }
    return false; // already initialized
}

bool CanOpen::loopbackTest()
{
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
    if (!queued)
    {
        addDataToOutQueue("Failed to queue test message for transmission");
        return false;
    }
    else
    {
        addDataToOutQueue("Test message queued for transmission");
    }

    delay(100); // Wait for message to loop back

    CAN_message_t receivedMsg;
    bool got = false;
    if (Can.read(receivedMsg))
    {
        got = true;
    }
    else
    {
        addDataToOutQueue("Failed to receive loopback message");
        return false;
    }

    addDataToOutQueue("Received loopback message with ID: " + String(receivedMsg.id, HEX));
    for (int i = 0; i < receivedMsg.len; ++i)
    {
        Serial2.print(receivedMsg.buf[i], HEX);
        Serial2.print(" ");
    }

    if (got && receivedMsg.id == testMsg.id && receivedMsg.len == testMsg.len)
    {
        bool dataMatch = true;
        for (int i = 0; i < testMsg.len; ++i)
        {
            if (receivedMsg.buf[i] != testMsg.buf[i])
            {
                dataMatch = false;
                break;
            }
        }
        if (dataMatch)
        {
            addDataToOutQueue("\nLoopback test successful");
            return true;
        }
        else
        {
            addDataToOutQueue("\nData mismatch in loopback test");
            return false;
        }
    }
    else
    {
        addDataToOutQueue("\nLoopback test failed: ID or length mismatch");
        return false;
    }
}

bool CanOpen::send(uint32_t id, const uint8_t *msgData, uint8_t msgDataLen) // data contains not only data but also SDO command specifier, index, subindex etc.
{
    // Check for null data pointer
    if (msgData == nullptr && msgDataLen > 0)
    {
        addDataToOutQueue("Error: Null data pointer in CAN send");
        return false;
    }

    // Check for invalid length (CAN frame can have max 8 bytes of data)
    if (msgDataLen > 8)
    {
        addDataToOutQueue("Error: Invalid data length in CAN send");
        return false;
    }

    CAN_TX_msg.id = id;
    CAN_TX_msg.flags.extended = 0;
    CAN_TX_msg.len = msgDataLen;

    // Copy data to CAN message buffer
    for (int i = 0; i < msgDataLen; ++i)
    {
        CAN_TX_msg.buf[i] = msgData[i];
    }

    // Zero out unused bytes in the buffer. Necessary, because the buffer may contain old data
    for (int i = msgDataLen; i < 8; ++i)
    {
        CAN_TX_msg.buf[i] = 0;
    }

    bool ok = Can.write(CAN_TX_msg);
    if (!ok)
    {
        addDataToOutQueue("CAN send failed for ID: " + String(id, HEX));
    }
    return ok;
}

bool CanOpen::receive(uint16_t &cob_id, uint8_t *data, uint8_t &len)
{
    if (Can.read(CAN_RX_msg))
    {
        cob_id = CAN_RX_msg.id;
        len = CAN_RX_msg.len;
        for (int i = 0; i < CAN_RX_msg.len; ++i)
        {
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

    if (receive(id, data, len))
    {
        uint16_t nodeId = id & 0x7F;         // Extract node ID from COB-ID
        if (nodeId <= 0 || RobotConstants::Robot::MAX_NODE_ID < nodeId)
        {
            addDataToOutQueue("Received message from invalid node ID: " + String(nodeId));
            return false;
        }
        
        uint16_t function_code = id & 0x780; // Extract base COB-ID

        if (function_code == RobotConstants::CANOpen::COB_ID_HEARTBEAT_BASE)
        {
            if (heartbeatCallback != nullptr)
            {
                heartbeatCallback(nodeId, data[0]);
            }
        }
        else if (function_code == RobotConstants::CANOpen::COB_ID_SDO_CLIENT_BASE)
        { // SDO READ/WRITE RESPONSE
            // addDataToOutQueue("SDO Response from node " + String(nodeId));

            // Accept 4-byte write acks and 8-byte read responses
            if (len < 4)
            {
                addDataToOutQueue("Invalid SDO response length from node " + String(nodeId) + ": " + String(len));
                return false;
            }

            uint16_t registerAddress = data[1] | (data[2] << 8);

            if (registerAddress == RobotConstants::ODIndices::ELECTRONIC_GEAR_MOLECULES)
            { // 0x260A
                if (electronicGearMoleculesCallbacks_260A[nodeId] != nullptr)
                {
                    electronicGearMoleculesCallbacks_260A[nodeId](nodeId, (data[0] == 0x60));
                }
            }
            else if (registerAddress == RobotConstants::ODIndices::CONTROLWORD)
            { // 0x6040
                if (controlWordCallbacks_6040[nodeId] != nullptr)
                {
                    controlWordCallbacks_6040[nodeId](nodeId, (data[0] == 0x60));
                }
            }
            else if (registerAddress == RobotConstants::ODIndices::MODES_OF_OPERATION)
            { // 0x6060
                if (modesOfOperationCallbacks_6060[nodeId] != nullptr)
                {
                    modesOfOperationCallbacks_6060[nodeId](nodeId, (data[0] == 0x60));
                }
            }
            else if (registerAddress == RobotConstants::ODIndices::TARGET_POSITION)
            { // 0x607A
                if (targetPositionCallbacks_607A[nodeId] != nullptr)
                {
                    targetPositionCallbacks_607A[nodeId](nodeId, (data[0] == 0x60));
                }
            }
            else if (registerAddress == RobotConstants::ODIndices::POSITION_ACTUAL_VALUE)
            { // 0x6064
                bool success = (data[0] != 0x80);
                int32_t positionValue = 0;
                if (success)
                {
                    positionValue = (static_cast<int32_t>(data[7]) << 24) |
                                    (static_cast<int32_t>(data[6]) << 16) |
                                    (static_cast<int32_t>(data[5]) << 8) |
                                    (static_cast<int32_t>(data[4]));
                }
                if (positionReadCallbacks_6064[nodeId] != nullptr)
                {
                    positionReadCallbacks_6064[nodeId](nodeId, success, positionValue);
                }
            }
            else if (registerAddress == RobotConstants::ODIndices::STATUSWORD)
            { // 0x6041
                if (statusWordCallbacks_6041[nodeId] != nullptr)
                {
                    bool success = (data[0] != 0x80);
                    uint16_t statusWordValue = 0;
                    if (success)
                    {
                        statusWordValue = static_cast<uint16_t>(data[4]) | (static_cast<uint16_t>(data[5]) << 8);
                    }
                    statusWordCallbacks_6041[nodeId](nodeId, success, statusWordValue);
                }
            }
            return false;
        }
        return true;
    }
    return false;
}
