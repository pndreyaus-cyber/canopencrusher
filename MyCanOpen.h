#ifndef MYCANOPEN_H
#define MYCANOPEN_H

#include <stdint.h>
#include <stddef.h>
#include "OD.h"
#include "MyCanDriver.h"
#include "objdict_objectdefines.h"
#include "CO/CO_driver.h"
#include "Arduino.h"

// Callback types for received SDO and PDO messages
//typedef void (*SdoReceiveCallback)(uint8_t nodeId, uint16_t index, uint8_t subindex, const uint8_t* data, uint8_t len);
//typedef void (*PdoReceiveCallback)(uint16_t cobId, const uint8_t* data, uint8_t len);


class MyCanOpen {
private:
    //MyCanDriver* Can;   
    CO_CANmodule_t* canModule;

public:
    MyCanOpen(CO_CANmodule_t *canModule = nullptr);

    bool send_x260A_electronicGearMolecules(uint8_t nodeId, int32_t value/*, */);
    bool send_x60FF_targetVelocity(uint8_t nodeId, int32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6081_profileVelocity(uint8_t nodeId, uint32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6040_controlword(uint8_t nodeId, uint16_t value /*, ODObjs_t * odobjs*/);
    bool send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value/*, ODObjs_t * odobjs*/);

    //bool sendSDO_legacy(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void* data);
    bool sendSDO(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void* data);
    bool sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute);
    bool sendSYNC();

    void writeReversedToBuf(const void* data, size_t size, uint8_t* bufStart);

    //void read(); // TODO: implement using CANopenNode

};

#endif