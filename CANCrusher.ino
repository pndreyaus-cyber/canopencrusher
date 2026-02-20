#include <unordered_set>
#include <queue>

#include <vector>
#include "STM32_CAN.h"
#include "CanOpenController.h"
#include "CanOpen.h"
#include "Params.h"
#include "RobotConstants.h"
#include "Debug.h"
#include "PrepareMoveTest.h"

HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

String inData;
uint8_t bufIndex = 0;       // хранилище данных с последовательного порта
std::queue<String> outData; // очередь сообщений на отправку

// Forward declarations
void stringToVelocityAndAcceleration(String paramsSubStr, MoveParams<RobotConstants::Robot::AXES_COUNT> &params, RobotConstants::MoveUnits moveUnits);
MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command, RobotConstants::MoveUnits moveUnits);
MotorIndices stringToMotorIndices(String command);

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, bool isAbsoluteMove);
void handleZeroInitialize(MotorIndices motorIndices);
void handleRequestPosition(MotorIndices motorIndices);
void handleMotorStatus(String command);

bool receiveCommand();
void handleCommand();
void addDataToOutQueue(String data);
void sendData();

bool isFloat(String str);

uint32_t lastTickTime_100 = 0;
uint32_t lastTickTime_500 = 0;

void setup()
{
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);

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
        {
        }
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
    inData.reserve(128); // Reserve space to avoid dynamic allocations during command reception
}

void loop()
{
    if (receiveCommand())
    {
        handleCommand();
    }

    sendData();
    canOpen.read();
    if (millis() - lastTickTime_100 >= 100)
    {
        lastTickTime_100 = millis();
        moveController.tick_100();
    }

    if (millis() - lastTickTime_500 >= 500)
    {
        lastTickTime_500 = millis();
        moveController.tick_500();
    }
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
        handleMove(stringToMoveParams(inData, RobotConstants::MoveUnits::UNITS_DEG), true);
    }

    else if (function.equals(RobotConstants::Commands::MOVE_RELATIVE))
    {
        handleMove(stringToMoveParams(inData, RobotConstants::MoveUnits::UNITS_DEG), false);
    }
    else if (function.equals(RobotConstants::Commands::ECHO))
    {
        addDataToOutQueue(inData.substring(4));
    }
    else if (function.equals(RobotConstants::Commands::MOTOR_STATUS))
    {
        handleMotorStatus(inData);
    }
    else if (function.equals(RobotConstants::Commands::ZERO_INITIALIZE))
    {
        handleZeroInitialize(stringToMotorIndices(inData));
    }
    else if (function.equals(RobotConstants::Commands::REQUEST_POSITION))
    {
        handleRequestPosition(stringToMotorIndices(inData));
    }
    else if (function.equals(RobotConstants::Commands::PREPAREMOVE_TEST))
    {
        handlePrepareMoveTest(inData);
    }
    else if (function.equals(RobotConstants::Commands::MOVE_ABSOLUTE_PERCENT))
    {
        handleMove(stringToMoveParams(inData, RobotConstants::MoveUnits::UNITS_PERCENT), true); // For now, treat MAP the same as MAJ. The move controller will need to be updated to handle percentage-based moves.
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
    if(outData.size() >= 100) // Limit the queue size to prevent memory issues
    {
        sendData();
    }
    outData.push(data);
    interrupts();
}

void sendData() // отправка сообщений на компьютер
{
    if (outData.empty())
    {
        return;
    }
    noInterrupts();
    String data = outData.front();
    outData.pop();
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

void stringToVelocityAndAcceleration(String paramsSubStr, MoveParams<RobotConstants::Robot::AXES_COUNT> &params, RobotConstants::MoveUnits moveUnits)
{
    int indexOfAC = paramsSubStr.indexOf("AC");
    if (indexOfAC == -1)
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "Expected acceleration parameter 'AC' after speed parameter";
        return;
    }

    String velocityStr = paramsSubStr.substring(2, indexOfAC);
    if (!isFloat(velocityStr))
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Invalid speed value: " + velocityStr;
        return;
    }

    float velocity = velocityStr.toFloat();

    String accelerationStr = paramsSubStr.substring(indexOfAC + 2);
    if (!isFloat(accelerationStr))
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Invalid acceleration value: " + accelerationStr;
        return;
    }
    float acceleration = accelerationStr.toFloat();

    if (moveUnits == RobotConstants::MoveUnits::UNITS_PERCENT)
    {
        if (velocity < 0.0f || velocity > 1.0f || acceleration < 0.0f || acceleration > 1.0f)
        {
            params.status = ParamsStatus::INVALID_PARAMS;
            params.errorMsg = "For percentage-based moves, speed and acceleration must be in the range [0, 1]: speed: " + String(velocity) + ", acceleration: " + String(acceleration);
            return;
        }
        params.speed = velocity;
        params.acceleration = acceleration;
    }
    else if (moveUnits == RobotConstants::MoveUnits::UNITS_DEG)
    {
        if (velocity < 0.0f || velocity > RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S || acceleration < 0.0f || acceleration > RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2)
        {
            params.status = ParamsStatus::INVALID_PARAMS;
            params.errorMsg = "For degree-based moves, speed must be in the range [0, " + String(RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S) + "] and acceleration must be in the range [0, " + String(RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2) + "]: speed: " + String(velocity) + ", acceleration: " + String(acceleration);
            return;
        }
        params.speed = velocity / RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S; // Convert to percentage of maximum velocity
        params.acceleration = acceleration / RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2; // Convert to percentage of maximum acceleration
    }
    else
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.errorMsg = "Invalid move units";
    }
}

MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command, RobotConstants::MoveUnits moveUnits)
{
    MoveParams<RobotConstants::Robot::AXES_COUNT> params;
    params.status = ParamsStatus::OK;

    String paramsStr = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space

    if (paramsStr.length() == 0)
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "No parameters provided";
        return params;
    }

    int i = 0;
    bool invalidParams = false;
    int nodeCnt = 0;
    while (i < paramsStr.length() && !invalidParams && paramsStr.charAt(i) == (char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) // Parse movement parameters until we reach speed parameter (starting with 'S')
    {
        char axisIdChar = paramsStr.charAt(i + 1);
        if (axisIdChar < RobotConstants::Robot::MIN_NODE_ID || RobotConstants::Robot::MAX_NODE_ID < axisIdChar)
        {
            params.errorMsg = "Invalid axis identifier: " + String(axisIdChar);
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
                    params.errorMsg = "Multiple decimal points in parameter for axis " + String(axisIdChar);
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
            params.errorMsg = "No numeric value provided for axis " + String(axisIdChar);
            invalidParams = true;
        }

        if (invalidParams)
        {
            break;
        }

        float movementUnits = paramsStr.substring(i + 2, j).toFloat();
        params.movementUnits[axisIdChar - RobotConstants::Robot::MIN_NODE_ID] = movementUnits; // Convert 'A'-'F' to 0-5 and store movement units
        nodeCnt++;
        i = j;
    }

    if (invalidParams)
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (nodeCnt == 0)
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "No movement parameters provided";
        return params;
    }

    if (paramsStr.substring(i, i + 2) != "SP")
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        params.errorMsg = "Expected speed parameter 'SP' at position " + String(i);
        return params;
    }

    stringToVelocityAndAcceleration(paramsStr.substring(i), params, moveUnits);
    if (params.status != ParamsStatus::OK)
    {
        return params;
    }
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

    if (isOk && i != params.length())
    {
        isOk = false;
        motorIndices.errorMsg = "Incomplete motor identifier at end of parameters";
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
        DBG_ERROR(DBG_GROUP_COMMAND, params.errorMsg);
        addDataToOutQueue((isAbsoluteMove ? RobotConstants::Commands::MOVE_ABSOLUTE : RobotConstants::Commands::MOVE_RELATIVE) + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }

    DBG_VERBOSE(DBG_GROUP_COMMAND, String(isAbsoluteMove ? "Handling absolute move command with parameters: " : "Handling relative move command with parameters: ") +
                                       "movements = " + String(RobotConstants::Robot::AXES_COUNT) + " nodes, " +
                                       "speed (RPM) = " + String(params.speed) + ", acceleration (RPM/s) = " + String(params.acceleration));
    // for(const auto& movement : params.movements) {
    //     DBG_VERBOSE(DBG_GROUP_COMMAND, "Node " + String(movement.first) + ": " + String(movement.second));
    // }

    if (!moveController.move(params))
    {
        addDataToOutQueue((isAbsoluteMove ? RobotConstants::Commands::MOVE_ABSOLUTE : RobotConstants::Commands::MOVE_RELATIVE) + " " + RobotConstants::Status::LOGIC_ERROR);
    }
}

void handleMotorStatus(String command)
{
    addDataToOutQueue("HANDLE MOTOR STATUS called");
    if (command != RobotConstants::Commands::MOTOR_STATUS)
    {
        DBG_VERBOSE(DBG_GROUP_COMMAND, RobotConstants::Commands::MOTOR_STATUS + " does not take any parameters");
        addDataToOutQueue(RobotConstants::Commands::MOTOR_STATUS + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    moveController.requestStatus();
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
    String reply = RobotConstants::Commands::REQUEST_POSITION + " " + RobotConstants::Status::OK + " ";
    for (uint8_t nodeId : motorIndices.nodeIds)
    {
        reply += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + String(moveController.axisPosition(nodeId)) + " ";
    }
    addDataToOutQueue(reply);
}

void handlePrepareMoveTest(String command)
{
    String params = command.substring(RobotConstants::Commands::COMMAND_LEN);
    bool isVerbose = false;
    if (params.length() > 0)
    {
        if (params.equals("V0"))
        {
            isVerbose = false;
        }
        else if (params.equals("V1"))
        {
            isVerbose = true;
        }
        else
        {
            addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_TEST + " " + RobotConstants::Status::INVALID_PARAMS);
            return;
        }
    }
    digitalWrite(PC13, LOW);
    delay(100);
    digitalWrite(PC13, HIGH);
    delay(100);
    digitalWrite(PC13, LOW);
    delay(100);
    digitalWrite(PC13, HIGH);
    
    bool success = runPrepareMoveTests(isVerbose);
    if (!success)
    {
        addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_TEST + " " + RobotConstants::Status::LOGIC_ERROR);
        return;
    }
    addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_TEST + " " + RobotConstants::Status::OK);
}