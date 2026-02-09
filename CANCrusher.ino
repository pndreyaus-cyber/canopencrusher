#include <unordered_set>

#include "STM32_CAN.h"
#include "CanOpenController.h"
#include "CanOpen.h"
#include "Params.h"
#include "RobotConstants.h"
#include "Debug.h"  

const double MAX_SPEED = 360;
const double MAX_ACCELERATION = 7864.20;

HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

String inData;
uint8_t bufIndex = 0;        // хранилище данных с последовательного порта
std::vector<String> outData; // очередь сообщений на отправку

// Forward declarations
MoveParams<RobotConstants::Robot::AXIS_COUNT> stringToMoveParams(String command);
MotorIndices stringToMotorIndices(String command);

void handleMove(MoveParams<RobotConstants::Robot::AXIS_COUNT> params, bool isAbsoluteMove);
void handleZeroInitialize(MotorIndices motorIndices);
void handleRequestPosition(MotorIndices motorIndices);
void handleMotorStatus(MotorIndices motorIndices);


void setup()
{
    Serial2.setRx(PA3);
    Serial2.setTx(PA2);

    Serial2.begin(115200);
    while (!Serial2)
    {
    }
    Serial2.println("Serial connected!");

    if (!canOpen.startCan(1000000))
    {
        Serial2.println("Failed to initialize CAN bus");
        while (1)
            ;
    }
    else
    {
        Serial2.println("CAN bus initialized successfully");
    }

    if (!moveController.start(&canOpen, RobotConstants::Robot::AXIS_COUNT))
    {
        Serial2.println("Failed to initialize MoveController");
        while (1)
            ;
    }
    else
    {
        Serial2.println("MoveController initialized successfully");
    }
    inData.reserve(128);
    outData.reserve(128);
}

void loop()
{
    if (receiveCommand())
        handleCommand();

    sendData();
    canOpen.read();
    moveController.tick();
}

bool receiveCommand()
{
    char received = 0x00;
    if (Serial2.available())
    {
        received = Serial2.read();
        inData += received;
    }
    return received == '\n';
}

void handleCommand()
{
    inData.replace(" ", "");
    inData.replace("\n", "");
    inData.replace("\r", "");

    if (inData.length() < 3)
    {
        addDataToOutQueue(inData + " " + RobotConstants::Status::INCORRECT_COMMAND);
        inData = "";
        return;
    }

    String function = inData.substring(0, 3);
    if (function.equals(RobotConstants::Commands::MOVE_ABSOLUTE))
    {
        handleMove(stringToMoveParams(inData), true);
    }

    else if (function.equals(RobotConstants::Commands::MOVE_RELATIVE))
    {
        handleMove(stringToMoveParams(inData), false);
    }
    else if (function.equals(RobotConstants::Commands::ECHO))
    {
        addDataToOutQueue(inData.substring(4));
    }
    else if (function.equals(RobotConstants::Commands::MOTOR_STATUS))
    {
        handleMotorStatus(stringToMotorIndices(inData));
    }
    else if (function.equals(RobotConstants::Commands::ZERO_INITIALIZE))
    {
        handleZeroInitialize(stringToMotorIndices(inData));
    }
    else if (function.equals(RobotConstants::Commands::REQUEST_POSITION))
    {
        handleRequestPosition(stringToMotorIndices(inData));
    }
    else
    {
        addDataToOutQueue(function + " " + RobotConstants::Status::INCORRECT_COMMAND);
    }
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

MoveParams<RobotConstants::Robot::AXIS_COUNT> stringToMoveParams(String command)
{
    MoveParams<RobotConstants::Robot::AXIS_COUNT> params;

    int JA_Index = command.indexOf("JA");
    int JB_Index = command.indexOf("JB");
    int JC_Index = command.indexOf("JC");
    int JD_Index = command.indexOf("JD");
    int JE_Index = command.indexOf("JE");
    int JF_Index = command.indexOf("JF");

    int speed_Index = command.indexOf("SP");
    int ACC_Index = command.indexOf("AC");

    if ((JA_Index == -1) || (JB_Index == -1) || (JC_Index == -1) || (JD_Index == -1) || (JE_Index == -1) || (JF_Index == -1) || (speed_Index == -1) || (ACC_Index == -1))
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


MotorIndices stringToMotorIndices(String command) 
{
    String params = command.substring(3); // Only parameters, without command and space
    MotorIndices motorIndices;
    motorIndices.status = ParamsStatus::OK;
    if (params.length() == 0)
    {
        return motorIndices;
    }

    int i = 0;
    bool isOk = true;
    while (i < params.length() - 1)
    {
        if (params.charAt(i) == 'J')
        {
            char motorChar = params.charAt(i + 1);
            if (motorChar >= 'A' && motorChar <= 'F')
            {
                motorIndices.nodeIds.push_back(motorChar - 'A' + 1); // Convert 'A'-'F' to 1-6
                i += 2;                                              // Skip the motor identifier
            }
            else
            {
                isOk = false;
                motorIndices.errorMsg = "Invalid motor identifier: " + String(motorChar);
                break;
            }
        }
        else
        {
            isOk = false;
            motorIndices.errorMsg = "Motor identifiers should start with 'J' followed by a letter";
            break;
        }
    }

    if (!isOk)
    {
        motorIndices.status = ParamsStatus::INVALID_PARAMS;
        motorIndices.errorCode = RobotConstants::Status::INVALID_PARAMS;
    }
    return motorIndices;
}

void handleMove(MoveParams<RobotConstants::Robot::AXIS_COUNT> params, bool isAbsoluteMove)
{
    if (params.status != ParamsStatus::OK)
    {
        if (params.status == ParamsStatus::INVALID_PARAMS)
        {
            addDataToOutQueue(RobotConstants::Commands::MOVE_ABSOLUTE + " " + RobotConstants::Status::INVALID_PARAMS);
        }
        else if (params.status == ParamsStatus::INCORRECT_COMMAND)
        {
            addDataToOutQueue(RobotConstants::Commands::MOVE_ABSOLUTE + " " + RobotConstants::Status::INCORRECT_COMMAND);
        }
        return;
    }

    for (uint8_t nodeId = 1; nodeId <= moveController.getAxesCount(); ++nodeId)
    {
        if (isAbsoluteMove)
            moveController.getAxis(nodeId).setTargetPositionAbsoluteInUnits(params.movementUnits[nodeId - 1]);
        else
            moveController.getAxis(nodeId).setTargetPositionRelativeInUnits(params.movementUnits[nodeId - 1]);
    }

    moveController.setRegularSpeedUnits(params.speed);
    moveController.setAccelerationUnits(params.acceleration);

    moveController.move();
}

void handleMotorStatus(MotorIndices motorIndices)
{
    if(motorIndices.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::MOTOR_STATUS + " " + motorIndices.errorCode + " " + motorIndices.errorMsg);
        return;
    }
    moveController.requestStatus(motorIndices.nodeIds);
}

void handleZeroInitialize(MotorIndices motorIndices)
{
    if (motorIndices.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::ZERO_INITIALIZE + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }

    if (motorIndices.nodeIds.size() == 0)
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, "Starting Zero Initialization for all nodes");
        moveController.startZeroInitializationAllAxes();
    }
    else if (motorIndices.nodeIds.size() == 1)
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, "Starting Zero Initialization for node " + String(motorIndices.nodeIds[0]));
        moveController.startZeroInitializationSingleAxis(motorIndices.nodeIds[0]);
    }
    else
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, "ZEI cupports only single axis initialization or all axes initialization");
        addDataToOutQueue(RobotConstants::Commands::ZERO_INITIALIZE + " " + RobotConstants::Status::INVALID_PARAMS);
    }
}

void handleRequestPosition(MotorIndices motorIndices) {
    if (motorIndices.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::REQUEST_POSITION + " " + motorIndices.errorCode + " " + motorIndices.errorMsg);
        return;
    }
    moveController.tick_requestPosition(motorIndices.nodeIds);
}