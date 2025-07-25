#ifndef MYCANOPEN_H
#define MYCANOPEN_H

#include <stdint.h>
#include <stddef.h>
#include "OD.h"
#include "MyCanDriver.h"
#include "objdict_objectdefines.h"

// Callback types for received SDO and PDO messages
//typedef void (*SdoReceiveCallback)(uint8_t nodeId, uint16_t index, uint8_t subindex, const uint8_t* data, uint8_t len);
//typedef void (*PdoReceiveCallback)(uint16_t cobId, const uint8_t* data, uint8_t len);


class MyCanOpen {
private:
    MyCanDriver* Can;
    ODObjs_t* ODObjs;
public:
    MyCanOpen(MyCanDriver* Can /*, void* canSendArg*/);

    bool send_x260A_electronicGearMolecules(uint8_t nodeId, int32_t value/*, */);
    bool send_x60FF_targetVelocity(uint8_t nodeId, int32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6083_profileAcceleration(uint8_t nodeId, uint32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6081_profileVelocity(uint8_t nodeId, uint32_t value/*, ODObjs_t * odobjs*/);
    bool send_x6040_controlword(uint8_t nodeId, uint16_t value /*, ODObjs_t * odobjs*/);
    bool send_x6060_modesOfOperation(uint8_t nodeId, uint8_t value/*, ODObjs_t * odobjs*/);

    bool sendSDO(uint8_t nodeId, uint8_t dataLen, uint16_t index, uint8_t subindex, void* data);
    bool sendPDO4_x607A_SyncMovement(uint8_t nodeId, int32_t targetPositionAbsolute);
    bool sendSYNC();
    // Send SDO write (expedited, littlen-endian)
    //bool sendSDOWrite(uint8_t nodeId, uint16_t index, uint8_t subindex, const void* data, uint8_t len);

    // Send PDO (up to 8 bytes)
    //bool sendPDO(uint16_t cobId, const void* data, uint8_t len);

    // Handle received CAN message
    //void handleReceived(const CAN_message_t& msg);

    // Register callback for received SDO messages
    //void registerSdoReceiveCallback(SdoReceiveCallback callback);

    // Register callback for received PDO messages
    //void registerPdoReceiveCallback(PdoReceiveCallback callback);

    void writeReversedToBuf(const void* data, size_t size, uint8_t* bufStart);

//private:
    //CanSendFunc _canSend;
    //void* _canSendArg;

    //SdoReceiveCallback _sdoCallback;
    //PdoReceiveCallback _pdoCallback;

    // Helper to build and send CAN message
    //bool sendCanMessage(uint32_t cobId, const uint8_t* data, uint8_t len);
};

//#include "MyCanOpen.tpp" // For template implementation if needed

#endif