#include "STM32_CAN.h"
#include "CO/CANopen.h" 
#include "OD.h"
#include "CO_driver_target.h"
#include "CanOpenController.h"
#include "MyCanDriver.h"
#include "Stm32CanDriver.h"
#include "MyCanOpen.h"
#include "Params.h"
#include <string.h>

#define STEPS_PER_REVOLUTION 32768
#define UNITS_PER_REVOLUTION 7.2

const String COMMAND_MOVE_ABSOLUTE = "MAJ";
const String COMMAND_MOVE_RELATIVE = "MRJ";
const String COMMAND_ECHO = "ECH";
const String COMMAND_SET_CURRENT_POSITION_IN_STEPS = "SCS";
const String COMMAND_SET_CURRENT_POSITION_IN_UNITS = "SCU";


const double MAX_SPEED = 360;
const double MAX_ACCELERATION = 7864.20;

// ======== For CANOpenNode ========
// CANopenNode CAN module and buffer objects
CO_CANmodule_t canModule;
CO_CANrx_t canRxArray[8]; // CAN receive buffer array
CO_CANtx_t canTxArray[8]; // CAN transmit buffer array
// Instantiate STM32_CAN object for CAN bus (CAN1 peripheral, ALT pins)
STM32_CAN Can(CAN1, ALT);
unsigned long lastSDO = 0;
static CAN_message_t CAN_TX_msg, CAN_RX_msg;

//======== For Motors ========
//HardwareSerial Serial2(PA3, PA2);

//STM32_CAN Can( CAN1, ALT );
//static CAN_message_t CAN_TX_msg;
//Stm32CanDriver can(1000000);
MyCanOpen canOpen(&canModule);
MoveController moveController(&canOpen);

const int axesNum = 6;
Axis axes[6] = {Axis(1), Axis(2), Axis(3), Axis(4), Axis(5), Axis(6)};

uint8_t buf[8];

String inData;
std::vector<String> outData; // очередь сообщений на отправку

MoveParams stringToMoveParams(String command) {
    MoveParams params;

    int JA_Index = command.indexOf("JA");
    int JB_Index = command.indexOf("JB");
    int JC_Index = command.indexOf("JC");
    int JD_Index = command.indexOf("JD");
    int JE_Index = command.indexOf("JE");
    int JF_Index = command.indexOf("JF");

    int speed_Index = command.indexOf("SP");
    int ACC_Index = command.indexOf("AC");

    if ((JA_Index == -1) || (JB_Index == -1) || (JC_Index == -1) || (JD_Index == -1) 
        || (JE_Index == -1) || (JF_Index == -1) || (speed_Index == -1) || (ACC_Index == -1)) {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        return params;
    }

    params.movementUnits[0] = command.substring(JA_Index + 2, JB_Index).toFloat();
    params.movementUnits[1] = command.substring(JB_Index + 2, JC_Index).toFloat();
    params.movementUnits[2] = command.substring(JC_Index + 2, JD_Index).toFloat();
    params.movementUnits[3] = command.substring(JD_Index + 2, JE_Index).toFloat();
    params.movementUnits[4] = command.substring(JE_Index + 2, JF_Index).toFloat();
    params.movementUnits[5] = command.substring(JF_Index + 2, speed_Index).toFloat();

    params.speed = command.substring(speed_Index + 2, ACC_Index).toFloat();
    params.acceleration = command.substring(ACC_Index + 2).toFloat();

    if ((params.speed <= 0.0F) || (params.acceleration <= 0.0F)) {
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (params.acceleration < 1.0F) {
        params.acceleration = 1.0F;
    }

    if (params.speed > 100.0F) {
        params.speed = 100.0F;
    }
    
    if (params.acceleration > 100.0F) {
        params.acceleration = 100.0F;
    }

    params.speed = MAX_SPEED * params.speed / 100.0F;
    params.acceleration = MAX_ACCELERATION * params.acceleration / 100.0F;

    params.status = ParamsStatus::OK;
    return params;
}

PositionParams stringToPositionParams(String command) {
    PositionParams params;

    int positionIndex = command.indexOf("POS");
    int idIndex = command.indexOf("ID");

    if ((positionIndex == -1) || (idIndex == -1)) {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        return params;
    }

    params.currentPosition = command.substring(positionIndex + 3, idIndex).toFloat();
    params.nodeId = command.substring(idIndex + 2).toInt();

    if ((params.nodeId <= 0.0F) || (params.nodeId > 6.0F)) {
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    params.status = ParamsStatus::OK;
    return params;
}

SDOParams stringToSDOParams(String command) {
    SDOParams params;

    int nodeIdIndex = command.indexOf("NID");
    int dataLenIndex = command.indexOf("LEN");
    int indexIndex = command.indexOf("INDEX");
    int subindexIndex = command.indexOf("SUB");
    int dataIndex = command.indexOf("DATA");

    if ((nodeIdIndex == -1) || (dataLenIndex == -1) || (indexIndex == -1) || (subindexIndex == -1) 
        || (dataIndex == -1)) {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        return params;
    }

    params.nodeId = command.substring(nodeIdIndex + 3, dataLenIndex).toInt();
    params.dataLen = command.substring(dataLenIndex + 3, indexIndex).toInt();
    params.index = command.substring(indexIndex + 5, subindexIndex).toInt();
    params.subindex = command.substring(subindexIndex + 3, dataIndex).toInt();
    
    // Parse the 64-bit data
    String dataStr = command.substring(dataIndex + 4);
    const char *s = dataStr.c_str();
    char *t;
    params.data = strtoul(s, &t, 0);
    if(s == t) {
        Serial2.println("strtoul failed!");
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }
    Serial2.print("params.data: ");
    Serial2.println(params.data);

    if ((params.nodeId <= 0) || (params.nodeId > 6) || (params.dataLen <= 0) || (params.dataLen > 4)) {
        Serial2.println("Node id or dataLen are invalid");
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    // Convert 64-bit data to 4-byte array (little-endian)
    Serial2.println();
    params.dataArray[0] = (uint8_t)(params.data & 0xFF);
    params.dataArray[1] = (uint8_t)((params.data >> 8) & 0xFF);
    params.dataArray[2] = (uint8_t)((params.data >> 16) & 0xFF);
    params.dataArray[3] = (uint8_t)((params.data >> 24) & 0xFF);
    Serial2.print("data in Hex: ");
    Serial2.println(params.data, HEX);
    for(int i = 0; i < 4; ++i){
        Serial2.print(params.dataArray[i]);
        Serial2.print(' ');
    }
    Serial2.println("");

    params.status = ParamsStatus::OK;
    Serial2.println("Returning correct params");
    return params;
}


void handleMove(MoveParams params, bool isAbsoluteMove) {
    if (params.status != ParamsStatus::OK) {
        if (params.status == ParamsStatus::INVALID_PARAMS) {
            addDataToOutQueue("INVALID PARAMS");
        } else if (params.status == ParamsStatus::INCORRECT_COMMAND) {
            addDataToOutQueue("INCORRECT COMMAND");
        }
        return;
    }
    
    for (int i = 0; i < axesNum; ++i) {
        if (isAbsoluteMove) {
            axes[i].setTargetPositionAbsoluteInUnits(params.movementUnits[i]);
        } else {
            axes[i].setTargetPositionRelativeInUnits(params.movementUnits[i]);
        }
    }

    moveController.setRegularSpeedUnits(params.speed);
    moveController.setAccelerationUnits(params.acceleration);

    moveController.moveAsync(axes[0], axes[1], axes[2], axes[3], axes[4], axes[5]);
}

void handleSetCurrentPositionInSteps(PositionParams params) {
    axes[params.nodeId].setCurrentPositionInSteps(params.currentPosition);
    addDataToOutQueue("(S)New current position for " + String(params.nodeId) + ": " + String(axes[params.nodeId].getCurrentPositionInSteps()));
}

void handleSetCurrentPositionInUnits(PositionParams params) {
    axes[params.nodeId].setCurrentPositionInUnits(params.currentPosition);
    addDataToOutQueue("(U)New current position for " + String(params.nodeId) + ": " + String(axes[params.nodeId].getCurrentPositionInSteps()));
}

void handleSendSimpleSDO(SDOParams params) {
    if (params.status != ParamsStatus::OK) {
        if (params.status == ParamsStatus::INVALID_PARAMS) {
            addDataToOutQueue("INVALID SDO PARAMS");
        } else if (params.status == ParamsStatus::INCORRECT_COMMAND) {
            addDataToOutQueue("INCORRECT SDO COMMAND");
        }
        return;
    }
    
    // Use the dataArray instead of the old malloc'd pointer
    Serial2.println("Entering canOpen.sendSDO");
    bool success = canOpen.sendSDO(params.nodeId, params.dataLen, params.index, params.subindex, params.dataArray);
    Serial2.println("Exited canOpen.sendSDO");

    if (success) {
        addDataToOutQueue("SDO SENT SUCCESSFULLY");
    } else {
        addDataToOutQueue("SDO SEND FAILED");
    }
}


void setup() {
    Serial2.begin(115200); 
    while (!Serial2) {}
    Serial2.println("Connected!");

    inData.reserve(128);
    outData.reserve(128);

    for (int i = 0; i < axesNum; ++i) {
        axes[i].setStepsPerRevolution(STEPS_PER_REVOLUTION);
        axes[i].setUnitsPerRevolution(UNITS_PER_REVOLUTION);
    }
    Serial2.print("Stopped Here?\n");

    // Initialize CANopenNode CAN driver (1000kbs)
    if(CO_CANmodule_init(&canModule, &Can, canRxArray, 8, canTxArray, 8, 1000000) != CO_ERROR_NO){ // 1000 = 1000kbps (adjust as needed)
        Serial2.println("CO_CANmodule_init: something wrong");
    }

    Serial2.println("Or here?");

    Serial2.println("CanOpenNode SDO client ready");
}

void loop() {
    if (receiveCommand()) {
        handleCommand();
    }

    sendData();
    //moveController.tick();

        // Process received messages
    CO_CANmodule_process(&canModule); // Handles incoming CAN messages and updates state

    // Process messages ready for sending
    CO_CANprocessTx(&canModule); // Sends any CAN messages queued in transmit buffers

    //delay(1000);
}

bool receiveCommand() {
    char received = 0x00;
    if (Serial2.available()) {
        received = Serial2.read();
        inData += received;
    }
    return received == '\n';
}

void handleCommand() {
    inData.replace(" ", "");
    inData.replace("\n", "");
    inData.replace("\r", "");

    if (inData.length() < 3) {
        inData = "";
        addDataToOutQueue("INVALID COMMAND: TOO SHORT");
        return;
    }

    String function = inData.substring(0, 3);

    if (function == COMMAND_MOVE_ABSOLUTE) {
        handleMove(stringToMoveParams(inData), true);
        addDataToOutQueue("MAJ COMMAND COMPLETED");
    } else if (function == COMMAND_MOVE_RELATIVE) {
        handleMove(stringToMoveParams(inData), false);
        addDataToOutQueue("MRJ COMMAND COMPLETED");
    } else if (function.equals(COMMAND_ECHO)) {
        addDataToOutQueue(inData.substring(4));  
    } else if (function == "SCS") {
        handleSetCurrentPositionInSteps(stringToPositionParams(inData));
        addDataToOutQueue("SCS COMMAND COMPLETED");
    } else if (function == "SCU") {
        handleSetCurrentPositionInUnits(stringToPositionParams(inData));
        addDataToOutQueue("SCU COMMAND COMPLETED");
    } else if (function == "SDO") {
        handleSendSimpleSDO(stringToSDOParams(inData));
        addDataToOutQueue("SDO COMMAND COMPLETED");
    } else {
        addDataToOutQueue("INVALID COMMAND");
    }

    inData = "";
}

void addDataToOutQueue(String data) { // добавление сообщений в очередь на отправку на компьютер
    noInterrupts();
    outData.push_back(data);
    interrupts();
}

void sendData() { // отправка сообщений на компьютер
    if (outData.size() == 0) {
        return;
    }

    noInterrupts();
    String data = outData.front();
    outData.erase(outData.begin());
    interrupts();

    Serial2.println(data);
}