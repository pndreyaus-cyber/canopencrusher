#include "STM32_CAN.h"
#include "CanOpenController.h"
#include "CanOpen.h"
#include "Params.h"

const String COMMAND_MOVE_ABSOLUTE = "MAJ";
const String COMMAND_MOVE_RELATIVE = "MRJ";
const String COMMAND_ECHO = "ECH";
const String COMMAND_SET_CURRENT_POSITION_IN_STEPS = "SCS";
const String COMMAND_SET_CURRENT_POSITION_IN_UNITS = "SCU";


const double MAX_SPEED = 360;
const double MAX_ACCELERATION = 7864.20;

HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

uint8_t buf[8];

String inData;
uint8_t bufIndex = 0;			 // хранилище данных с последовательного порта
std::vector<String> outData; // очередь сообщений на отправку

MoveParams stringToMoveParams(String command)
{
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
  || (JE_Index == -1) || (JF_Index == -1) || (speed_Index == -1) || (ACC_Index == -1) )
	{
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

	if ((params.speed <= 0.0F) || (params.acceleration <= 0.0F))
	{
		params.status = ParamsStatus::INVALID_PARAMS;
		return params;
	}

	if (params.acceleration < 1.0F)
		params.acceleration = 1.0F;

	if (params.speed > 100.0F)
		params.speed = 100.0F;
	
	if (params.acceleration > 100.0F)
		params.acceleration = 100.0F;

	params.speed = MAX_SPEED * params.speed / 100.0F;
	params.acceleration = MAX_ACCELERATION * params.acceleration / 100.0F;

	params.status = ParamsStatus::OK;
	return params;
}

PositionParams stringToPositionParams(String command)
{
	PositionParams params;

	int positionIndex = command.indexOf("POS");
	int idIndex = command.indexOf("ID");

	if ((positionIndex == -1) || (idIndex == -1) )
	{
		params.status = ParamsStatus::INCORRECT_COMMAND;
		return params;
	}

  params.currentPosition = command.substring(positionIndex + 3, idIndex).toFloat();
  params.nodeId = command.substring(idIndex + 2).toInt();

	if ((params.nodeId <= 0.0F) || (params.nodeId > 6.0F))
	{
		params.status = ParamsStatus::INVALID_PARAMS;
		return params;
	}

	params.status = ParamsStatus::OK;
	return params;
}


void handleMove(MoveParams params, bool isAbsoluteMove){
    if(params.status != ParamsStatus::OK){
        if(params.status == ParamsStatus::INVALID_PARAMS)
            addDataToOutQueue("INVALID PARAMS");
        else if(params.status == ParamsStatus::INCORRECT_COMMAND)
            addDataToOutQueue("INCORRECT COMMAND");
        return;
    }
    
    for(int i = 0; i < moveController.getAxisNum(); ++i){

        if(isAbsoluteMove) moveController.getAxis(i).setTargetPositionAbsoluteInUnits(params.movementUnits[i]);
        else moveController.getAxis(i).setTargetPositionRelativeInUnits(params.movementUnits[i]);
    }

    moveController.setRegularSpeedUnits(params.speed);
    moveController.setAccelerationUnits(params.acceleration);

    moveController.moveAbsolute();
    //moveController.moveAsync(axes[0], axes[1], axes[2], axes[3], axes[5]); // TODO: не забыть про ось 4
}

void handleSetCurrentPositionInSteps(PositionParams params){
    moveController.getAxis(params.nodeId).setCurrentPositionInSteps(params.currentPosition);
    addDataToOutQueue("(S)New current position for " + String(params.nodeId) + ": " + String(moveController.getAxis(params.nodeId).getCurrentPositionInSteps()));
}

void handleSetCurrentPositionInUnits(PositionParams params){
    moveController.getAxis(params.nodeId).setCurrentPositionInUnits(params.currentPosition);
    addDataToOutQueue("(U)New current position for " + String(params.nodeId) + ": " + String(moveController.getAxis(params.nodeId).getCurrentPositionInSteps()));
}


void setup() {
    Serial2.setRx(PA3);
    Serial2.setTx(PA2);

    Serial2.begin(115200); 
    while(!Serial2){}
    Serial2.println("Serial connected!");

    if (!canOpen.startCan(1000000)) {
        Serial2.println("Failed to initialize CAN bus");
        while (1);
    } else {
        Serial2.println("CAN bus initialized successfully");
    }
    
    moveController.start(&canOpen, 6);

    // for (int i = 0; i < axesNum; ++i) {
    //     axes[i].setMotorId(i + 1);
    // }

    inData.reserve(128);
    outData.reserve(128);

}

void loop() {
    if (receiveCommand())
        handleCommand();

    sendData();
}

bool receiveCommand()
{
    char received = 0x00;
    if(Serial2.available()){
        received = Serial2.read();
        inData += received;
    }
    return received == '\n';
}

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