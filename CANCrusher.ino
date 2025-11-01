// Include CANopenNode and STM32 CAN library headers
#include "CO_driver_target.h"
//#include "CO/CO_driver.h" // Uncomment if needed for your platform
#include "OD.h" // Object Dictionary
//#include "CO_storageBlank.h" // Uncomment if needed
#include "CO/CANopen.h" // Main CANopenNode header
#include "STM32_CAN.h" // STM32 CAN library

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
#define STEPS_PER_REVOLUTION 32768
#define UNITS_PER_REVOLUTION 7.2

const String COMMAND_MOVE_ABSOLUTE = "MAJ";
const String COMMAND_MOVE_RELATIVE = "MRJ";
const String COMMAND_ECHO = "ECH";
const String COMMAND_SET_CURRENT_POSITION_IN_STEPS = "SCS";
const String COMMAND_SET_CURRENT_POSITION_IN_UNITS = "SCU";


const double MAX_SPEED = 360;
const double MAX_ACCELERATION = 7864.20;


//MoveController moveController(&canOpen);




void sendSDO_6060(uint8_t nodeId) {
    // Prepare SDO message to write 0x05 to index 0x6060, subindex 0
    uint16_t can_id = 0x600 + nodeId; // SDO client->server command
    uint8_t data[8] = {0x2B, 0x0A, 0x26, 0x00, 0x66, 0xEA, 0x05, 0x00}; // SDO write command for 0x6060

    // Find a free transmit buffer
    for (uint8_t i = 0; i < 8; i++) {
        if (!canTxArray[i].bufferFull) {
            canTxArray[i].ident = can_id;
            canTxArray[i].DLC = 8;
            memcpy(canTxArray[i].data, data, 8);
            canTxArray[i].syncFlag = false;

            // Queue the message for sending
            CO_ReturnError_t err = CO_CANsend(&canModule, &canTxArray[i]);
            if (err == CO_ERROR_NO) {
                Serial2.println("SendSDO_6060 success!");
            } else {
                Serial2.println("SendSDO_6060 error!");
            }
            break;
        }
    }
}


void setup() {
    // Initialize serial for debugging
    Serial2.begin(115200); 
    while(!Serial2){} // Wait for serial port to connect
    delay(2);
    Serial2.println("CAN connected!");

    // Initialize CANopenNode CAN driver (1000kbs)
    if(CO_CANmodule_init(&canModule, &Can, canRxArray, 8, canTxArray, 8, 1000000) != CO_ERROR_NO){ // 1000 = 1000kbps (adjust as needed)
        Serial2.println("CO_CANmodule_init: something wrong");
    }

    Serial2.println("CanOpenNode SDO client ready");
}

void loop() {
    // Process received messages
    CO_CANmodule_process(&canModule); // Handles incoming CAN messages and updates state

    // Process messages ready for sending
    CO_CANprocessTx(&canModule); // Sends any CAN messages queued in transmit buffers

    if (millis() - lastSDO >= 5000) {
        sendSDO_6060(1); // Node ID 1
        lastSDO = millis();
    }
    

    delay(1000); // Increased delay for easier serial monitoring

}

/*
bool receiveCommand()
{
	char received = 0x00;
	if(Serial2.available()){
		received = Serial2.read();
		inData += received;
	}
	return received == '\n';
}
*/

/*
void handleCommand()
{
	inData.replace(" ","");
	inData.replace("\n", "");
	inData.replace("\r", "");

	if (inData.length() < 3)
	{
		inData = "";
		addDataToOutQueue("INVALID COMMAND: TOO SHORT");
		return;
	}

	String function = inData.substring(0, 3);

	if (function == COMMAND_MOVE_ABSOLUTE)
	{
    handleMove(stringToMoveParams(inData), true);
    addDataToOutQueue("MAJ COMMAND COMPLETED");
	}

	else if (function == COMMAND_MOVE_RELATIVE)
	{
    handleMove(stringToMoveParams(inData), false);
    addDataToOutQueue("MRJ COMMAND COMPLETED");
	}
	else if (function.equals(COMMAND_ECHO))
    addDataToOutQueue(inData.substring(4));  
  else if(function == "SCS")
  {
    handleSetCurrentPositionInSteps(stringToPositionParams(inData));
		addDataToOutQueue("SCS COMMAND COMPLETED");
  }
  else if(function == "SCU")
  {
    handleSetCurrentPositionInUnits(stringToPositionParams(inData));
		addDataToOutQueue("SCU COMMAND COMPLETED");
  }
  else
    addDataToOutQueue("INVALID COMMAND");

	inData = "";
}

void addDataToOutQueue(String data) // добавление сообщений в очередь на отправку на компьютер
{
	noInterrupts();
	outData.push_back(data);
	interrupts();
}

void sendData() // отправка сообщений на компьютер
{
	if (outData.size() == 0)
		return;

	noInterrupts();
	String data = outData.front();
	outData.erase(outData.begin());
	interrupts();

	Serial2.println(data);
}
*/