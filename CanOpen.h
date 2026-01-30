#ifndef CAN_OPEN_H

#define CAN_OPEN_H

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include "OD.h"
#include "objdict_objectdefines.h"
#include "STM32_CAN.h"

// Make SDOReceiveCallback type
using SdoReadPositionCallback = std::function<void(uint8_t, int32_t)>;

class CanOpen {
private:
    STM32_CAN Can;
    CAN_message_t CAN_TX_msg;
    CAN_message_t CAN_RX_msg;
    bool can_initialized = false;
    bool loopbackTest();
    uint32_t canBaudRate;

    //ODObjs_t* ODObjs = nullptr;

    SdoReadPositionCallback sdoReadPositionCallback = nullptr;

    bool send(uint32_t id, const uint8_t* data, uint8_t len);
    bool receive(uint16_t &cob_id, uint8_t* data, uint8_t &len);

public:
    CanOpen() : Can(PA11, PA12) {};
    bool startCan(uint32_t baudRate);

    bool send_zeroInitialize(uint8_t nodeId);
    bool send_x260A_electronicGearMolecules(uint8_t nodeId, int32_t value/*, */);
    bool send_x60FF_targetVelocity(uint8_t nodeId, int32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6081_profileVelocity(uint8_t nodeId, uint32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6040_controlword(uint8_t nodeId, uint16_t value /*, ODObjs_t * odobjs*/);
    bool send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value/*, ODObjs_t * odobjs*/);

    bool sendSDO(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void* data);
    bool sendSDORead(uint8_t nodeId, uint16_t index, uint8_t subindex);
    bool sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute);
    bool sendSYNC();

    void writeReversedToBuf(const void* data, size_t size, uint8_t* bufStart);

    void setSdoReadPositionCallback(SdoReadPositionCallback callback) {
        sdoReadPositionCallback = callback;
    }

    uint8_t read(); // TODO: implement using CANopenNode

};

#endif