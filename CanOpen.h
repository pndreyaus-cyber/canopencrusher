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

    callback_x6064_positionActualValue callbacks_x6064_positionActualValue[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};         // index 0 is unused
    callback_x260A_electronicGearMolecules callbacks_x260A_electronicGearMolecules[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr}; // index 0 is unused
    callback_x6040_controlword callbacks_x6040_controlword[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                         // index 0 is unused
    callback_x6060_modesOfOperation callbacks_x6060_modesOfOperation[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};               // index 0 is unused
    callback_x607A_targetPosition callbacks_x607A_targetPosition[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                   // index 0 is unused
    callback_x6081_profileVelocity callbacks_x6081_profileVelocity[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                 // index 0 is unused
    callback_x6083_profileAcceleration callbacks_x6083_profileAcceleration[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};         // index 0 is unused
    callback_TPDO1 callbacks_TPDO1[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                                                 // index 0 is unused
    callback_TPDO4 callbacks_TPDO4[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};                                                 // index 0 is unused

    callback_x2614_dataSaveFlag callbacks_x2614_dataSaveFlag = nullptr;

    callback_heartbeat callbacks_heartbeat = nullptr;

    callback_read_x6041_statusword callbacks_read_x6041_statusword[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr};   // index 0 is unused
    callback_read_x6040_controlword callbacks_read_x6040_controlword[RobotConstants::Robot::AXES_COUNT + 1] = {nullptr}; // index 0 is unused

    callback_PI_controller callbacks_PI_controller = nullptr;

    callback_read_PI_controller callbacks_read_PI_controller = nullptr; // Won't call simultaneously for multiple nodes, so we don't need an array
    callback_read_x2614_dataSaveFlag callbacks_read_x2614_dataSaveFlag = nullptr;

public:
    CanOpen() : Can(PB8, PB9, RX_SIZE_128, TX_SIZE_128) {};
    bool startCan(uint32_t baudRate);

    bool send_x260A_electronicGearMolecules(uint8_t nodeId, uint16_t value);
    bool send_x60FF_targetVelocity(uint8_t nodeId, int32_t value);
    bool send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value);
    bool send_x6081_profileVelocity(uint8_t nodeId, uint32_t value);
    bool send_x6040_controlword(uint8_t nodeId, uint16_t value);
    bool send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value);
    bool send_x607A_targetPosition(uint8_t nodeId, int32_t value);
    bool send_RPDO1(uint8_t nodeId, uint16_t controlWord, int8_t workMode, int32_t targetPosition);
    bool send_RPDO4(uint8_t nodeId, int32_t targetPositionAbsolute);
    bool x60F9_velocityControlParameterSet_PGain(uint8_t nodeId, int16_t value);
    bool x60F9_velocityControlParameterSet_IGain(uint8_t nodeId, int16_t value);
    bool x60FB_positionControlParameterSet_PGain(uint8_t nodeId, int16_t value);
    bool x60FB_positionControlParameterSet_FeedForwardFactor(uint8_t nodeId, int16_t value);
    bool saveParameters(uint8_t nodeId);
    bool sendSYNC();

    bool sendSDOWrite(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, const void *data);
    bool sendSDORead(uint8_t nodeId, uint16_t index, uint8_t subindex);
    void set_callback_x260A_electronicGearMolecules(callback_x260A_electronicGearMolecules callback, uint8_t nodeId)
    {
        callbacks_x260A_electronicGearMolecules[nodeId] = callback;
    }

    void set_callback_x6040_controlword(callback_x6040_controlword callback, uint8_t nodeId)
    {
        callbacks_x6040_controlword[nodeId] = callback;
    }

    void set_callback_x6060_modesOfOperation(callback_x6060_modesOfOperation callback, uint8_t nodeId)
    {
        callbacks_x6060_modesOfOperation[nodeId] = callback;
    }

    void set_callback_x6064_positionActualValue(callback_x6064_positionActualValue callback, uint8_t nodeId)
    {
        callbacks_x6064_positionActualValue[nodeId] = callback;
    }

    void set_callback_x607A_targetPosition(callback_x607A_targetPosition callback, uint8_t nodeId)
    {
        callbacks_x607A_targetPosition[nodeId] = callback;
    }

    void set_callback_x6081_profileVelocity(callback_x6081_profileVelocity callback, uint8_t nodeId)
    {
        callbacks_x6081_profileVelocity[nodeId] = callback;
    }

    void set_callback_x6083_profileAcceleration(callback_x6083_profileAcceleration callback, uint8_t nodeId)
    {
        callbacks_x6083_profileAcceleration[nodeId] = callback;
    }

    void set_callback_TPDO1(callback_TPDO1 callback, uint8_t nodeId)
    {
        callbacks_TPDO1[nodeId] = callback;
    }

    void set_callback_TPDO4(callback_TPDO4 callback, uint8_t nodeId)
    {
        callbacks_TPDO4[nodeId] = callback;
    }

    void set_callback_heartbeat(callback_heartbeat callback)
    {
        callbacks_heartbeat = callback;
    }

    void set_callback_read_x6041_statusword(callback_read_x6041_statusword callback, uint8_t nodeId)
    {
        callbacks_read_x6041_statusword[nodeId] = callback;
    }

    void set_callback_read_x6040_controlword(callback_read_x6040_controlword callback, uint8_t nodeId)
    {
        callbacks_read_x6040_controlword[nodeId] = callback;
    }

    void set_callback_PI_controller(callback_PI_controller callback)
    {
        callbacks_PI_controller = callback;
    }

    void set_callback_read_PI_controller(callback_read_PI_controller callback)
    {
        callbacks_read_PI_controller = callback;
    }

    void set_callback_x2614_dataSaveFlag(callback_x2614_dataSaveFlag callback)
    {
        callbacks_x2614_dataSaveFlag = callback;
    }

    void set_callback_read_x2614_dataSaveFlag(callback_read_x2614_dataSaveFlag callback)
    {
        callbacks_read_x2614_dataSaveFlag = callback;
    }

    bool read();
};

#endif