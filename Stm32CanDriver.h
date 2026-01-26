#ifndef STM32CANDRIVER_H
#define STM32CANDRIVER_H
#include "MyCanDriver.h"
#include "STM32_CAN.h"
#include "Params.h"

class Stm32CanDriver : public MyCanDriver {
private:
    STM32_CAN Can;
    CAN_message_t CAN_TX_msg;
    CAN_message_t CAN_RX_msg;
    bool initialized = false;
    bool loopbackTest();
    uint32_t baudRate;
public:
    Stm32CanDriver() : Can(PA11, PA12) {};
    //Stm32CanDriver(uint32_t baudRate);
    ~Stm32CanDriver();    

    bool start(uint32_t baudRate);

    bool send(uint32_t id, const uint8_t* data, uint8_t len) override;
    bool receive(uint32_t &id, uint8_t* data, uint8_t &len) override;
};


#endif