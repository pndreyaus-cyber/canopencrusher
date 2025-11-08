#ifndef MYCAN_H
#define MYCAN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t buf[8];
    struct {
        bool extended;
        bool remote;
    } flags;
    uint8_t bus;
} My_CAN_message_t;

// Tell wrapper which CAN instance to use
void MyCAN_setInstance(void* CANptr);

void MyCAN_setConfigurationMode(void* CANptr);
void MyCAN_setNormalMode(void* CANptr);
void MyCAN_configureAndBeginCan(uint16_t CANbitRate);
void MyCAN_CANmodule_disable();
//void MyCAN_write(uint32_t id, uint8_t* data, uint8_t len);
//bool MyCAN_read(uint32_t* id, uint8_t* data, uint8_t* len);
bool MyCAN_write(uint32_t id, const uint8_t* data, uint8_t len);
bool MyCAN_read(My_CAN_message_t* msg);

#ifdef __cplusplus
}
#endif

#endif // MYCAN_H
