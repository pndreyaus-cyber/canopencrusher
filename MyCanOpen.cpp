#include "MyCanOpen.h"
#include <cstring> // for memcpy



MyCanOpen::MyCanOpen(CO_CANmodule_t *canModule) : canModule(canModule)
{
}

bool MyCanOpen::send_x260A_electronicGearMolecules(uint8_t nodeId, int32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _ElectronicGearMolecules_Idx,
        _ElectronicGearMolecules_ElectronicGearMolecules_sIdx,
        &value
    );
}

bool MyCanOpen::send_x60FF_targetVelocity(uint8_t nodeId, int32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _Target_velocity_Idx,
        _Target_velocity_Target_velocity_sIdx,
        &value
    );
}

bool MyCanOpen::send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _Profile_acceleration_Idx,
        _Profile_acceleration_Profile_acceleration_sIdx,
        &value
    );
}

bool MyCanOpen::send_x6081_profileVelocity(uint8_t nodeId, uint32_t value)
{
    return sendSDO(
        nodeId,
        4,
        _Profile_velocity_Idx,
        _Profile_velocity_Profile_velocity_sIdx,
        &value
    );
}

bool MyCanOpen::send_x6040_controlword(uint8_t nodeId, uint16_t value)
{
    return sendSDO(
        nodeId,
        2,
        _Controlword_Idx,
        _Controlword_Controlword_sIdx,
        &value
    );
}

bool MyCanOpen::send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value)
{
    return sendSDO(
        nodeId,
        1,
        _Modes_of_operation_Idx,
        _Modes_of_operation_Modes_of_operation_sIdx,
        &value
    );    
}
/*
bool MyCanOpen::sendSDO_legacy(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void *data)
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
    return Can->send(0x600 + nodeId, msgBuf, dataLen + 4);
}
*/

bool MyCanOpen::sendSDO(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void* data) // Check if it corectly reverse bytes???
{
    Serial2.println("MyCanOpen::sendSDO");
    uint16_t can_id = 0x600 + nodeId; // SDO client->server command
    uint8_t msgBuf[8] = {0};
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

    CO_CANtx_t *txArray = canModule->txArray;
    CO_CANrx_t *rxArray = canModule->rxArray;
    // Find a free transmit buffer
    bool sent = false;
    Serial2.println("I am here?");
    for (uint8_t i = 0; i < 8; i++) {
        Serial2.print("MyCanOPen I: ");
        Serial2.println(i);
        if (!txArray[i].bufferFull) {
            txArray[i].ident = can_id;
            txArray[i].DLC = 8;
            memcpy(txArray[i].data, msgBuf, 8);
            txArray[i].syncFlag = false;

            // Queue the message for sending
            CO_ReturnError_t err = CO_CANsend(canModule, &txArray[i]);
            if (err == CO_ERROR_NO) {
                Serial2.println("SendSDO success!");
                sent = true;
            } else {
                Serial2.println("herehere SDO SendSDO error!");
                sent = true;
                Serial2.println("not here\n");
            }
            break;
        }
        if(sent) break;
    }
    Serial2.println("MyCanOpen::sendSDO exit!");
    return true;
}


bool MyCanOpen::sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute)
{
    //uint8_t msgBuf[4] = {0};
    //memcpy(msgBuf, &targetPositionAbsolute, 4);
    //return Can->send(0x500 + nodeId, msgBuf, 4);

    uint16_t can_id = 0x500 + nodeId; // SDO client->server command
    uint8_t msgBuf[4] = {0};
    memcpy(msgBuf, &targetPositionAbsolute, 4);

    CO_CANtx_t *txArray = canModule->txArray;
    CO_CANrx_t *rxArray = canModule->rxArray;
    // Find a free transmit buffer
    for (uint8_t i = 0; i < 8; i++) {
        if (!txArray[i].bufferFull) {
            txArray[i].ident = can_id;
            txArray[i].DLC = 4;
            memset(txArray[i].data, 0, 8 * sizeof(uint8_t));
            memcpy(txArray[i].data, msgBuf, 4);
            txArray[i].syncFlag = false;

            // Queue the message for sending
            CO_ReturnError_t err = CO_CANsend(canModule, &txArray[i]);
            if (err == CO_ERROR_NO) {
                Serial2.println("SendSDO success!");
                return true;
            } else {
                Serial2.println("6070A SendSDO error!");
                return false;
            }
            break;
        }
    }
}

bool MyCanOpen::sendSYNC()
{
    uint16_t can_id = 0x80; // SDO client->server command

    CO_CANtx_t *txArray = canModule->txArray;
    CO_CANrx_t *rxArray = canModule->rxArray;
    // Find a free transmit buffer
    for (uint8_t i = 0; i < 8; i++) {
        if (!txArray[i].bufferFull) {
            txArray[i].ident = can_id;
            txArray[i].DLC = 0;
            //memcpy(txArray[i].data, msgBuf, 4);
            memset(txArray[i].data, 0, 8 * sizeof(uint8_t));
            txArray[i].syncFlag = false;

            // Queue the message for sending
            CO_ReturnError_t err = CO_CANsend(canModule, &txArray[i]);
            if (err == CO_ERROR_NO) {
                Serial2.println("SendSDO success!");
                return true;
            } else {
                Serial2.println("80 SYNC SendSDO error!");
                return false;
            }
            break;
        }
    }
}

void MyCanOpen::writeReversedToBuf(const void* data, size_t size, uint8_t* bufStart) {
    const uint8_t* src_bytes = static_cast<const uint8_t*>(data);
    
    for (size_t i = 0; i < size; ++i) {
        bufStart[i] = src_bytes[size - 1 - i];  // Reverse copy
    }
}