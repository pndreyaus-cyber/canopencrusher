#ifndef MYCANDRIVER_H
#define MYCANDRIVER_H
#include <cstdint>

class MyCanDriver {
public:

    virtual ~MyCanDriver() = default;
    // Send a CAN message (id, data pointer, length)
    virtual bool send(uint32_t id, const uint8_t* data, uint8_t len) = 0;

    // Try to receive a CAN message (blocking or non-blocking depends on implementation)
    // Returns true if a message was received, false otherwise; sets id, data, length
    virtual bool receive(uint32_t &id, uint8_t* data, uint8_t &len) = 0;

};

#endif