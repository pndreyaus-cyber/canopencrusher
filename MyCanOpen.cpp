#include "MyCanOpen.h"
#include <cstring> // for memcpy
//#include <Arduino.h> // for Serial print, remove if not using Arduino


/*
MyCanOpen::MyCanOpen(CanSendFunc canSend, void* canSendArg)
    : _canSend(canSend), _canSendArg(canSendArg), _sdoCallback(nullptr), _pdoCallback(nullptr)
{
}
*/

MyCanOpen::MyCanOpen(MyCanDriver* Can) : Can(Can)
{
}

/*
bool MyCanOpen::sendCanMessage(uint32_t cobId, const uint8_t* data, uint8_t len)
{
    if (len > 8) return false;
    CAN_message_t msg;
    msg.id = cobId;
    msg.len = len;
    memcpy(msg.data, data, len);
    if (_canSend) {
        _canSend(&msg, _canSendArg);
        return true;
    }
    return false;
}

bool MyCanOpen::sendSDOWrite(uint8_t nodeId, uint16_t index, uint8_t subindex, const void* data, uint8_t len)
{
    if (len > 4 || len == 0) return false; // expedited SDO max 4 bytes
    uint8_t sdoData[8] = {0};
    // SDO command specifier for expedited write: 0x23, 0x2B, 0x27, 0x2F depending on length
    // Calculate command specifier
    uint8_t n = 4 - len; // number of unused bytes
    sdoData[0] = 0x23 | (n << 2); // command specifier
    sdoData[1] = index & 0xFF;
    sdoData[2] = (index >> 8) & 0xFF;
    sdoData[3] = subindex;
    memcpy(&sdoData[4], data, len);
    uint32_t cobId = 0x600 + nodeId; // SDO client to server
    return sendCanMessage(cobId, sdoData, 8);
}

bool MyCanOpen::sendPDO(uint16_t cobId, const void* data, uint8_t len)
{
    if (len > 8) return false;
    return sendCanMessage(cobId, (const uint8_t*)data, len);
}

void MyCanOpen::handleReceived(const CAN_message_t& msg)
{
    uint32_t id = msg.id;
    const uint8_t* data = msg.data;
    uint8_t len = msg.len;

    // Check if SDO message (server to client or client to server)
    // SDO server to client: 0x580 + nodeId
    // SDO client to server: 0x600 + nodeId
    // For simplicity, assume nodeId = 1
    uint8_t nodeId = 1;
    if (id == (0x580 + nodeId) || id == (0x600 + nodeId)) {
        if (_sdoCallback && len >= 8) {
            uint16_t index = data[1] | (data[2] << 8);
            uint8_t subindex = data[3];
            _sdoCallback(nodeId, index, subindex, &data[4], len - 4);
        }
    }
    // Check if PDO message (0x180 to 0x5FF)
    else if (id >= 0x180 && id <= 0x5FF) {
        if (_pdoCallback) {
            _pdoCallback((uint16_t)id, data, len);
        }
    }
    // Other message types can be added later
}


void MyCanOpen::registerSdoReceiveCallback(SdoReceiveCallback callback)
{
    _sdoCallback = callback;
}

void MyCanOpen::registerPdoReceiveCallback(PdoReceiveCallback callback)
{
    _pdoCallback = callback;
}

template<typename T>
void MyCanOpen::toCanOpenLE(const T& value, uint8_t* out)
{
    for (size_t i = 0; i < sizeof(T); ++i) {
        out[i] = (value >> (8 * i)) & 0xFF;
    }
}
    */

// Explicit template instantiation for common types
/*
void MyCanOpen::toCanOpenLE16(const uint16_t& value, uint8_t* out){
    for (size_t i = 0; i < sizeof(uint16_t); ++i) {
        out[i] = (value >> (8 * i)) & 0xFF;
    }
}
void MyCanOpen::toCanOpenLE32(const uint32_t& value, uint8_t* out){
    for (size_t i = 0; i < sizeof(uint32_t); ++i) {
        out[i] = (value >> (8 * i)) & 0xFF;
    }   
}
*/

/*
bool MyCanOpen::send_x260A_electronicGearMolecules(uint8_t nodeId, ODObjs_t* odobjs)
{
    return sendSDO(
        nodeId,
        odobjs->o_260A_electronicGearMolecules.dataLength,
        _ElectronicGearMolecules_Idx,
        _ElectronicGearMolecules_ElectronicGearMolecules_sIdx,
        odobjs->o_260A_electronicGearMolecules.dataOrig
    );
}
*/

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

bool MyCanOpen::sendSDO(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void *data)
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
    //toCanOpenLE16(index, msgBuf[1]);
    //writeReversedToBuf(&index, 2, msgBuf + 1);
    memcpy(&msgBuf[1], &index, 2);
    msgBuf[3] = subindex;
    //writeReversedToBuf(data, dataLen, msgBuf + 4);
    memcpy(&msgBuf[4], data, dataLen);

    // Отправляем (если ваша библиотека использует CAN.write или queue)
    return Can->send(0x600 + nodeId, msgBuf, dataLen + 4);
}

bool MyCanOpen::sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute)
{
    uint8_t msgBuf[4] = {0};
    memcpy(msgBuf, &targetPositionAbsolute, 4);
    return Can->send(0x500 + nodeId, msgBuf, 4);
}

bool MyCanOpen::sendSYNC()
{
    return Can->send(0x80, 0, 0);
}

void MyCanOpen::writeReversedToBuf(const void* data, size_t size, uint8_t* bufStart) {
    const uint8_t* src_bytes = static_cast<const uint8_t*>(data);
    
    for (size_t i = 0; i < size; ++i) {
        bufStart[i] = src_bytes[size - 1 - i];  // Reverse copy
    }
}