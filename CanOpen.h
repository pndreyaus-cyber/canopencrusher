#ifndef CAN_OPEN_H

#define CAN_OPEN_H

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include "OD.h"
#include "objdict_objectdefines.h"
#include "STM32_CAN.h"
#include "RobotConstants.h"

// Make SDOReceiveCallback type

extern void addDataToOutQueue(String data);

class CanOpen
{
private:
    STM32_CAN Can;
    CAN_message_t CAN_TX_msg;
    CAN_message_t CAN_RX_msg;
    bool can_initialized = false;
    bool loopbackTest();
    uint32_t canBaudRate;

    bool send(uint32_t id, const uint8_t *data, uint8_t len);
    bool receive(uint16_t &cob_id, uint8_t *data, uint8_t &len);

    callback_x6064_positionActualValue positionReadCallbacks_6064[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                // index 0 is unused
    callback_x260A_electronicGearMolecules electronicGearMoleculesCallbacks_260A[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr}; // index 0 is unused
    callback_x6040_controlword controlWordCallbacks_6040[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                         // index 0 is unused
    callback_x6060_modesOfOperation modesOfOperationCallbacks_6060[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};               // index 0 is unused
    callback_x607A_targetPosition targetPositionCallbacks_607A[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                   // index 0 is unused
    callback_x6041_statusword statusWordCallbacks_6041[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                           // index 0 is unused
    callback_heartbeat heartbeatCallback = nullptr;

public:
    CanOpen() : Can(PA11, PA12, RX_SIZE_128, TX_SIZE_128) {};
    bool startCan(uint32_t baudRate);

    bool send_x260A_electronicGearMolecules(uint8_t nodeId, uint16_t value);
    bool send_x60FF_targetVelocity(uint8_t nodeId, int32_t value);
    bool send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value);
    bool send_x6081_profileVelocity(uint8_t nodeId, uint32_t value);
    bool send_x6040_controlword(uint8_t nodeId, uint16_t value);
    bool send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value);
    bool send_x607A_targetPosition(uint8_t nodeId, int32_t value);

    bool sendSDOWrite(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, const void *data);
    bool sendSDORead(uint8_t nodeId, uint16_t index, uint8_t subindex);
    bool sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute);
    bool sendSYNC();

    void setElectronicGearMoleculesWriteStatusCallback_0x260A(callback_x260A_electronicGearMolecules callback, uint8_t nodeId)
    {
        electronicGearMoleculesCallbacks_260A[nodeId] = callback;
    }

    void setControlWordWriteStatusCallback_0x6040(callback_x6040_controlword callback, uint8_t nodeId)
    {
        controlWordCallbacks_6040[nodeId] = callback;
    }

    void setModesOfOperationWriteStatusCallback_0x6060(callback_x6060_modesOfOperation callback, uint8_t nodeId)
    {
        modesOfOperationCallbacks_6060[nodeId] = callback;
    }

    void setPositionActualValueCallback_0x6064(callback_x6064_positionActualValue callback, uint8_t nodeId)
    {
        positionReadCallbacks_6064[nodeId] = callback;
    }

    void setTargetPositionWriteStatusCallback_0x607A(callback_x607A_targetPosition callback, uint8_t nodeId)
    {
        targetPositionCallbacks_607A[nodeId] = callback;
    }

    void setStatusWordCallback_0x6041(callback_x6041_statusword callback, uint8_t nodeId)
    {
        statusWordCallbacks_6041[nodeId] = callback;
    }

    void setHeartbeatCallback(callback_heartbeat callback)
    {
        heartbeatCallback = callback;
    }

    bool read();
};

#endif