// TODO: Restore include path to "301/CO_driver.h" when using a proper build system
#include <Arduino.h>
#include <STM32_CAN.h>
#include "CO/CO_driver.h"

// Assume you have a global CAN object, e.g.:
// extern STM32_CAN CAN;

// Set CAN to configuration mode
void CO_CANsetConfigurationMode(void* CANptr) {
    STM32_CAN* can = (STM32_CAN*)CANptr;
    can->setMode(STM32_CAN::NORMAL);
}

// Set CAN to normal mode
void CO_CANsetNormalMode(void* CANptr) {
    STM32_CAN* can = (STM32_CAN*)CANptr;
    can->setMode(STM32_CAN::NORMAL);
}

// Initialize CAN module
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize,
                                   CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    Serial2.println("Line 32");
    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = (rxSize <= 32U) ? true : false; /* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
    
    Serial2.println("Line42");
    for (i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }

    for (i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }
    Serial2.println("Line55");
    /* Configure CAN module */
    STM32_CAN* can = (STM32_CAN*)CANptr;
    //can->setAutoRetransmission(true);
    can->begin();
    can->setBaudRate(1000000);
    Serial2.println("Line60");

    CAN_message_t CAN_TX_msg;
    CAN_TX_msg.id = (0x601);
    CAN_TX_msg.len = 6;
    CAN_TX_msg.buf[0] =  0x2B;
    CAN_TX_msg.buf[1] =  0x40;
    CAN_TX_msg.buf[2] =  0x60;
    CAN_TX_msg.buf[3] =  0x00;
    CAN_TX_msg.buf[4] =  0x0F;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0x00;

    Serial2.println("Line73");

    bool t = can->write(CAN_TX_msg);
    Serial2.print("Initialized Status: ");
    Serial2.println(t);

    Serial2.println("CAN module initialized!!!");
    return CO_ERROR_NO;

    /* Configure CAN module hardware filters */
    if (CANmodule->useCANrxFilters) {
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    } else {
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
    }

    /* configure CAN interrupt registers */
    Serial2.println("Line100");
}

void CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    if (CANmodule != NULL) {
        STM32_CAN* can = (STM32_CAN*)CANmodule->CANptr;
        can->end();
    }
}

/**
 * Configure CAN message receive buffer.
 *
 * Function configures specific CAN receive buffer. It sets CAN identifier and connects buffer with specific object.
 * Function must be called for each member in _rxArray_ from CO_CANmodule_t.
 *
 * Return #CO_ReturnError_t: CO_ERROR_NO CO_ERROR_ILLEGAL_ARGUMENT or CO_ERROR_OUT_OF_MEMORY (not enough masks for
 * configuration).
 */
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask,
                                    bool_t rtr, void* object, void (*CANrx_callback)(void* object, void* message)){
    
    CO_ReturnError_t ret = CO_ERROR_NO;
    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize)) {
        /* buffer, which will be configured */
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if (rtr) {
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {}
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer, microcontroller specific. */
        // IMPORTANT: buffer->ident is not just CAN ID. It has also DLC and RTR
        buffer->ident = ((uint32_t)ident & 0x07FFU) | ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 11U))
                        | ((uint32_t)(rtr ? 0x8000U : 0U));
        
        /*
        Example of extracting fields from buffer->ident

        uint16_t ident = buffer->ident;

        uint16_t can_id = ident & 0x07FFU;

        uint8_t dlc = (ident >> 11) & 0x0FU;

        bool rtr = (ident & 0x8000U) != 0;
        */

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);

    // If a synchronous TPDO is currently being inhibited, clear the inhibit flag.
    if (CANmodule->bufferInhibitFlag) {
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
        // On STM32 with Arduino, you do not need to clear a hardware TXREQ bit here.
    }

    // Clear any pending synchronous TPDOs in the software TX buffer.
    if (CANmodule->CANtxCount != 0U) {
        uint16_t i;
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--) {
            if (buffer->bufferFull && buffer->syncFlag) {
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;
                tpdoDeleted = 2U;
            }
            buffer++;
        }
    }

    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/*
// Send CAN message
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {
    if (CANmodule == NULL || buffer == NULL) return CO_ERROR_ILLEGAL_ARGUMENT;

    STM32_CAN* can = (STM32_CAN*)CANmodule->CANptr;
    CAN_message_t tx_msg;
    tx_msg.id = buffer->ident;
    tx_msg.len = buffer->DLC;
    tx_msg.ext = 0; // 0 for standard frame, set to 1 if using extended IDs

    for (uint8_t i = 0; i < tx_msg.len; i++) {
        tx_msg.buf[i] = buffer->data[i];
    }

    if (can->write(tx_msg)) {
        return CO_ERROR_NO;
    } else {
        return CO_ERROR_TX_OVERFLOW;
    }
}
*/

void CO_CANprocessTx(CO_CANmodule_t* CANmodule) {
    // Serial2.print("Enter CO_CANprocessTx: ");
    // Serial2.println(CANmodule->CANtxCount);
    if (CANmodule->CANtxCount > 0) {
        // Serial2.print("CANmodule->txSize = ");
        // Serial2.println(CANmodule->txSize);
        for (uint16_t i = 0; i < CANmodule->txSize; i++) {
            CO_CANtx_t* buffer = &CANmodule->txArray[i];
            if (buffer->bufferFull) {
                // Unpack CANopenNode's ident field
                uint16_t can_id = buffer->ident & 0x07FFU;
                uint8_t dlc = (buffer->ident >> 11) & 0x0FU;
                bool rtr = (buffer->ident & 0x8000U) != 0;

                CAN_message_t tx_msg;
                tx_msg.id = can_id;
                tx_msg.len = dlc;
                tx_msg.flags.remote = rtr ? 1 : 0;
                tx_msg.flags.extended = 0; // Standard frame

                for (uint8_t j = 0; j < tx_msg.len; j++) {
                    tx_msg.buf[j] = buffer->data[j];
                }

                STM32_CAN* can = (STM32_CAN*)CANmodule->CANptr;
                // Serial2.println("SERIOUSLY SENDING!!!");
                if (can->write(tx_msg)) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    // Serial2.println("Sent SDO!");
                    break; // Only send one message per call
                }else{
                    // Serial2.println("Error while sending");
                }
            }
        }
    }
}


CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    // Serial2.println("CO_CANsend");

    // Verify overflow
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            // don't set error, if bootup message is still on buffers 
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    //CO_LOCK_CAN_SEND(CANmodule);

    // If CAN TX buffer is free, send immediately
    if (CANmodule->CANtxCount == 0) {
        //Serial2.println("Sending immediately");
        CANmodule->bufferInhibitFlag = buffer->syncFlag;

        // Unpack CANopenNode's ident field
        uint16_t can_id = buffer->ident;
        uint8_t dlc = buffer->DLC;

        CAN_message_t tx_msg;
        tx_msg.id = buffer->ident;
        tx_msg.len = buffer->DLC;
        tx_msg.flags.extended = 0; // Standard frame

        for (uint8_t i = 0; i < tx_msg.len; i++) {
            tx_msg.buf[i] = buffer->data[i];
        }

        STM32_CAN* can = (STM32_CAN*)CANmodule->CANptr;
        if (!can->write(tx_msg)) {
            err = CO_ERROR_TX_OVERFLOW;
            Serial2.println("Error while sending");
        } else {
            CANmodule->firstCANtxMessage = false;
            Serial2.println("CO_driver.cpp Sent SDO!");
        }
    } else { // if no buffer is free, message will be sent by interrupt
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    //CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}




// Receive CAN message (to be called in your main loop or interrupt)
void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    //Serial2.println("CO_CANmodule_process");
    STM32_CAN* can = (STM32_CAN*)CANmodule->CANptr;
    CAN_message_t rx_msg;

    if(can->read(rx_msg)) {
        Serial2.print("Channel:");
        Serial2.print(rx_msg.bus);
        if (rx_msg.flags.extended == false) {
        Serial2.print(" Standard ID:");
        }
        else {
        Serial2.print(" Extended ID:");
        }
        Serial2.print(rx_msg.id, HEX);

        Serial2.print(" DLC: ");
        Serial2.print(rx_msg.len);
        if (rx_msg.flags.remote == false) {
        Serial2.print(" buf: ");
        for(int i=0; i<rx_msg.len; i++) {
            Serial2.print("0x"); 
            Serial2.print(rx_msg.buf[i], HEX); 
            if (i != (rx_msg.len-1))  Serial2.print(" ");
        }
        Serial2.println();
        } else {
        Serial2.println(" Data: REMOTE REQUEST FRAME");
        }
    }

/*
    // Poll for received CAN messages
    while (can->read(rx_msg)) {
        Serial2.print("New incoming message!: ");
        Serial.print(rx_msg.id, HEX);
        Serial2.println((rx_msg.id & 0x07FFU) | (0x0800U));
        for (uint16_t i = 0; i < CANmodule->rxSize; i++) {
            CO_CANrx_t *rxArray = &CANmodule->rxArray[i];
            if (((rx_msg.id ^ rxArray->ident) & rxArray->mask) == 0) {
                // Call the registered callback if set
                if (rxArray->CANrx_callback) {
                    rxArray->CANrx_callback(rxArray->object, &rx_msg);
                }
            }
        }
    }
*/
}

// Receive CAN message (to be called in your main loop or interrupt)
/*
void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    STM32_CAN* can = (STM32_CAN*)CANmodule->CANptr;
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    while (can->receive(&id, data, &len)) {
        for (uint16_t i = 0; i < CANmodule->rxSize; i++) {
            CO_CANrx_t *rxArray = &CANmodule->rxArray[i];
            if (((id ^ rxArray->ident) & rxArray->mask) == 0) {
                // Call callback if set
                if (rxArray->pFunct) {
                    rxArray->pFunct(rxArray->object, data);
                }
            }
        }
    }
}
    */