#ifndef MYCAN_H
#define MYCAN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Tell wrapper which CAN instance to use
void MyCAN_setInstance(void* CANptr);

void MyCAN_setConfigurationMode(void* CANptr);
void MyCAN_setNormalMode(void* CANptr);
void MyCAN_write(uint32_t id, uint8_t* data, uint8_t len);
bool MyCAN_read(uint32_t* id, uint8_t* data, uint8_t* len);

#ifdef __cplusplus
}
#endif

#endif // MYCAN_H
