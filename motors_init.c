//#include "CO.h"          // core CANopen types
#include "OD_instances.h" // the one you just made
#include "CO_driver.h"   // your CAN driver
#include "CANopen.h"

CO_t CO_Motors[MAX_MOTORS];

void motors_init(CO_CANmodule_t *sharedCAN) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        // prepare per-motor OD
        OD_create_instance(i);                 // from OD_instances.c
        init_OD_RAM(&OD_RAM_instances[i]);        // your per-motor RAM init
        init_ODObjs(&ODObjs_instances[i], &OD_RAM_instances[i]); // if you added that helper
        init_ODList(&OD_instances[i], &ODObjs_instances[i]);    // your per-motor OD list init

        // initialize CANopen for this motor
        uint8_t nodeId = i + 1;
        CO_CANopenInit(&CO_Motors[i], &OD_Motors[i], nodeId);
        CO_Motors[i].CANmodule = sharedCAN;
    }
}
