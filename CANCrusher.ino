#include <unordered_set>

#include "STM32_CAN.h"
#include "CanOpenController.h"
#include "CanOpen.h"
#include "Params.h"
#include "RobotConstants.h"
#include "Debug.h"

HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

String inData;
uint8_t bufIndex = 0;        // хранилище данных с последовательного порта
std::vector<String> outData; // очередь сообщений на отправку

// Forward declarations
MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command);
MotorIndices stringToMotorIndices(String command);

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, bool isAbsoluteMove);
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

    if (!moveController.start(&canOpen, RobotConstants::Robot::AXES_COUNT))
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

bool isFloat(String str)
{
    int i = 0;
    if (str.charAt(0) == '-' || str.charAt(0) == '+')
    {
        i = 1; // Skip sign if present
    }
    bool decimalPointFound = false;
    for (; i < str.length(); ++i)
    {
        char c = str.charAt(i);
        if (c == '.')
        {
            if (decimalPointFound)
                return false; // More than one decimal point
            decimalPointFound = true;
        }
        else if (!isDigit(c))
        {
            return false; // Non-digit character found
        }
    }
    return true; // String is a valid float
}

MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command)
{
    MoveParams<RobotConstants::Robot::AXES_COUNT> params;

    String paramsStr = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space

    if (paramsStr.length() == 0)
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "No parameters provided";
        return params;
    }

    int i = 0;
    int nodeCnt = 0;
    bool invalidParams = false;
    while (i < paramsStr.length() && nodeCnt < RobotConstants::Robot::AXES_COUNT && !invalidParams)
    {
        String axisIdentifier = String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeCnt));

        if (!paramsStr.substring(i, i + 2).equals(axisIdentifier))
        {
            params.errorMsg = "Expected " + axisIdentifier + " at position " + String(i);
            invalidParams = true;
            break;
        }

        int j = i + 2;
        bool decimalPointFound = false;
        if (paramsStr.charAt(j) == '-' || paramsStr.charAt(j) == '+')
            j++; // Skip sign if present

        while (j < paramsStr.length() && !invalidParams)
        {
            char c = paramsStr.charAt(j);
            if (c == '.')
            {
                if (decimalPointFound)
                {
                    params.errorMsg = "Multiple decimal points in parameter for " + axisIdentifier;
                    invalidParams = true;
                    break;
                }
                decimalPointFound = true;
            }
            else if (!isDigit(c))
            {
                break;
            }
            j++;
        }
        if (j == i + 2)
        {
            params.errorMsg = "No numeric value provided for " + axisIdentifier;
            invalidParams = true;
        }

        if (invalidParams)
        {
            break;
        }

        float movementUnits = paramsStr.substring(i + 2, j).toFloat();
        params.movementUnits[nodeCnt] = movementUnits;

        i = j;
        nodeCnt++;
    }

    if (invalidParams)
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (nodeCnt < RobotConstants::Robot::AXES_COUNT)
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "Expected parameters for " + String(RobotConstants::Robot::AXES_COUNT) + " axes, but got " + String(nodeCnt);
        return params;
    }

    if (paramsStr.substring(i, i + 2) != "SP")
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "Expected speed parameter 'SP' at position " + String(i);
        return params;
    }

    int indexOfAC = paramsStr.indexOf("AC", i);
    if (indexOfAC == -1)
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "Expected acceleration parameter 'AC' after speed parameter";
        return params;
    }

    String velocityStr = paramsStr.substring(i + 2, indexOfAC);
    if (!isFloat(velocityStr))
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Invalid speed value: " + velocityStr;
        return params;
    }

    params.speed = velocityStr.toFloat();
    if (params.speed <= RobotConstants::Commands::MIN_SPEED_UNITS || RobotConstants::Commands::MAX_SPEED_UNITS < params.speed)
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Speed must be in the range (" + String(RobotConstants::Commands::MIN_SPEED_UNITS) + ", " + String(RobotConstants::Commands::MAX_SPEED_UNITS) + "]: " + String(params.speed);
        return params;
    }

    String accelerationStr = paramsStr.substring(indexOfAC + 2);
    if (!isFloat(accelerationStr))
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Invalid acceleration value: " + accelerationStr;
        return params;
    }

    params.acceleration = accelerationStr.toFloat();
    if (params.acceleration <= RobotConstants::Commands::MIN_ACCELERATION_UNITS || RobotConstants::Commands::MAX_ACCELERATION_UNITS < params.acceleration)
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Acceleration must be in the range (" + String(RobotConstants::Commands::MIN_ACCELERATION_UNITS) + ", " + String(RobotConstants::Commands::MAX_ACCELERATION_UNITS) + "]: " + String(params.acceleration);
        return params;
    }

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
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            motorIndices.nodeIds.push_back(nodeId);
        }
        return motorIndices;
    }

    int i = 0;
    bool isOk = true;
    while (i < params.length() - 1)
    {
        if (params.charAt(i) != RobotConstants::Robot::AXIS_IDENTIFIER_CHAR)
        {
            isOk = false;
            motorIndices.errorMsg = "Motor identifiers should start with '" + String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + "' followed by a letter";
            break;
        }

        char motorChar = params.charAt(i + 1);
        if (motorChar < RobotConstants::Robot::MIN_NODE_ID || motorChar > RobotConstants::Robot::MAX_NODE_ID)
        {
            isOk = false;
            motorIndices.errorMsg = "Invalid motor identifier: " + String(motorChar);
            break;
        }

        uint8_t nodeId = (motorChar - RobotConstants::Robot::MIN_NODE_ID) + 1;
        if (nodeId > RobotConstants::Robot::AXES_COUNT)
        {
            isOk = false;
            motorIndices.errorMsg = "Motor identifier out of range: " + String(motorChar);
            break;
        }

        motorIndices.nodeIds.push_back(nodeId); // Convert 'A'-'F' to 1-6
        i += 2;                                 // Skip the motor identifier
    }

    if (!isOk)
    {
        motorIndices.status = ParamsStatus::INVALID_PARAMS;
        motorIndices.errorCode = RobotConstants::Status::INVALID_PARAMS;
    }
    return motorIndices;
}

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, bool isAbsoluteMove)
{
    if (params.status != ParamsStatus::OK)
    {
        DBG_ERROR(DBG_GROUP_MOVE, params.errorMsg);
        addDataToOutQueue((isAbsoluteMove ? RobotConstants::Commands::MOVE_ABSOLUTE : RobotConstants::Commands::MOVE_RELATIVE) + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }

    DBG_VERBOSE(DBG_GROUP_MOVE, String(isAbsoluteMove ? "Handling absolute move command with parameters: " : "Handling relative move command with parameters: ") +
                                    "movementUnits=[" + String(params.movementUnits[0]) + ", " + String(params.movementUnits[1]) + ", " + String(params.movementUnits[2]) + ", " + String(params.movementUnits[3]) + ", " + String(params.movementUnits[4]) + "], " +
                                    "speed=" + String(params.speed) + ", acceleration=" + String(params.acceleration));

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
    if (motorIndices.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::MOTOR_STATUS + " " + motorIndices.errorMsg);
        addDataToOutQueue(RobotConstants::Commands::MOTOR_STATUS + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    moveController.requestStatus(motorIndices.nodeIds);
}

void handleZeroInitialize(MotorIndices motorIndices)
{
    if (motorIndices.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::ZERO_INITIALIZE + " " + motorIndices.errorMsg);
        addDataToOutQueue(RobotConstants::Commands::ZERO_INITIALIZE + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    if (motorIndices.nodeIds.size() == RobotConstants::Robot::AXES_COUNT)
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, RobotConstants::Commands::ZERO_INITIALIZE + " Starting Zero Initialization for all nodes");
        moveController.startZeroInitializationAllAxes();
    }
    else if (motorIndices.nodeIds.size() == 1)
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, RobotConstants::Commands::ZERO_INITIALIZE + " Starting Zero Initialization for node " + String(motorIndices.nodeIds[0]));
        moveController.startZeroInitializationSingleAxis(motorIndices.nodeIds[0]);
    }
    else
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, RobotConstants::Commands::ZERO_INITIALIZE + " ZEI supports only single axis initialization or all axes initialization");
        addDataToOutQueue(RobotConstants::Commands::ZERO_INITIALIZE + " " + RobotConstants::Status::INVALID_PARAMS);
    }
}

void handleRequestPosition(MotorIndices motorIndices)
{
    if (motorIndices.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::REQUEST_POSITION + " " + motorIndices.errorMsg);
        addDataToOutQueue(RobotConstants::Commands::REQUEST_POSITION + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    moveController.tick_requestPosition(motorIndices.nodeIds);
}