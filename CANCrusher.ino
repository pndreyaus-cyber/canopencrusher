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

//HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

String inData;
uint8_t bufIndex = 0;       // хранилище данных с последовательного порта
std::queue<String> outData; // очередь сообщений на отправку

// Forward declarations
void stringToVelocityAndAcceleration(String paramsSubStr, MoveParams<RobotConstants::Robot::AXES_COUNT> &params, RobotConstants::MoveUnits moveUnits);
MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command, RobotConstants::MoveUnits moveUnits);
MotorIndices stringToMotorIndices(String command);

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, const String &command, bool isAbsoluteMove);
void handleZeroInitialize(MotorIndices motorIndices);
void handleRequestPosition(MotorIndices motorIndices);
void handleRequestPositionAngles(MotorIndices motorIndices);
void handleMotorStatus(String params);
void handlePIControl(String params);
void handleRequestPI(MotorIndices motorIndices);

bool receiveCommand();
void handleCommand();
void addDataToOutQueue(String data);
void sendData();

bool isFloat(String str);

uint32_t lastTickTime_100 = 0;
uint32_t lastTickTime_500 = 0;

void setup()
{
    // pinMode(PC13, OUTPUT);
    // digitalWrite(PC13, HIGH);


    Serial.begin(115200);
    while (!Serial)
    {
    }
    Serial.println("SER OK");

    if (!canOpen.startCan(1000000))
    {
        Serial.println("COP FF");
        while (1)
        {
        }
    }
    else
    {
        Serial.println("COP OK");
    }

    uint8_t nodesToInvert[] = {3, 4};    
    ParamsStatusStruct moveControllerInitStatus = moveController.start(&canOpen, RobotConstants::Robot::AXES_COUNT, true, nodesToInvert, 2); 
    if (moveControllerInitStatus.status == ParamsStatus::INVALID_PARAMS)
    {
        Serial.println("MVC FF " + moveControllerInitStatus.errorMsg.value_or("no error message"));
        while (1)
            ;
    }
    else
    {
        Serial.println("MVC OK");
    }
    inData.reserve(128); // Reserve space to avoid dynamic allocations during command reception
    Serial.println("Setup complete!!!!");
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
    if (Serial.available())
    {
        received = Serial.read();
        inData += received;
    }
    return received == '\n';
}

void handleCommand()
{
    inData.replace(" ", "");
    inData.replace("\n", "");
    inData.replace("\r", "");

    if (inData.length() < RobotConstants::Commands::COMMAND_LEN)
    {
        addDataToOutQueue(inData + " " + RobotConstants::Status::INCORRECT_COMMAND);
        inData = "";
        return;
    }

    String function = inData.substring(0, RobotConstants::Commands::COMMAND_LEN);
    if (function.equals(RobotConstants::Commands::MOVE_ABSOLUTE))
    {   
        addDataToOutQueue(RobotConstants::Commands::MOVE_ABSOLUTE + " " + RobotConstants::Status::NOT_IMPLEMENTED);
        //handleMove(stringToMoveParams(inData, RobotConstants::MoveUnits::UNITS_DEG), RobotConstants::Commands::MOVE_ABSOLUTE, true);
    }

    else if (function.equals(RobotConstants::Commands::MOVE_RELATIVE))
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_RELATIVE + " " + RobotConstants::Status::NOT_IMPLEMENTED);
        //handleMove(stringToMoveParams(inData, RobotConstants::MoveUnits::UNITS_DEG), RobotConstants::Commands::MOVE_RELATIVE, false);
    }
    else if (function.equals(RobotConstants::Commands::ECHO))
    {
        addDataToOutQueue(RobotConstants::Commands::ECHO + " " + inData.substring(4));
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
    else if (function.equals(RobotConstants::Commands::REQUEST_POSITION_ANGLES))
    {
        handleRequestPositionAngles(stringToMotorIndices(inData));
    }
    else if (function.equals(RobotConstants::Commands::PREPAREMOVE_TEST))
    {
        handlePrepareMoveTest(inData);
    }
    else if (function.equals(RobotConstants::Commands::MOVE_ABSOLUTE_PERCENT))
    {
        handleMove(stringToMoveParams(inData, RobotConstants::MoveUnits::UNITS_PERCENT), RobotConstants::Commands::MOVE_ABSOLUTE_PERCENT, true); // For now, treat MAP the same as MAJ. The move controller will need to be updated to handle percentage-based moves.
    }
    else if (function.equals(RobotConstants::Commands::PI_CONTROL))
    {
        handlePIControl(inData);
    }
    else if (function.equals(RobotConstants::Commands::REQUEST_PI))
    {
        handleRequestPI(stringToMotorIndices(inData));
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
    if (outData.size() >= RobotConstants::Buffers::SERIAL_OUT_QUEUE_CAPACITY) // Limit the queue size to prevent memory issues
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

    Serial.println(data);
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
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "Expected acceleration parameter 'AC' after speed parameter";
        return;
    }

    String velocityStr = paramsSubStr.substring(2, indexOfAC);
    if (!isFloat(velocityStr))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "Invalid speed value: " + velocityStr;
        return;
    }

    float velocity = velocityStr.toFloat();

    String accelerationStr = paramsSubStr.substring(indexOfAC + 2);
    if (!isFloat(accelerationStr))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "Invalid acceleration value: " + accelerationStr;
        return;
    }

    float acceleration = accelerationStr.toFloat();

    if (moveUnits == RobotConstants::MoveUnits::UNITS_PERCENT)
    {
        if (velocity < 0.0f || velocity > 1.0f || acceleration < 0.0f || acceleration > 1.0f)
        {
            params.status.status = ParamsStatus::INVALID_PARAMS;
            params.status.errorMsg = "For percentage-based moves, speed and acceleration must be in the range [0, 1]: speed: " + String(velocity) + ", acceleration: " + String(acceleration);
            return;
        }
        params.speed = velocity;
        params.acceleration = acceleration;
    }
    else if (moveUnits == RobotConstants::MoveUnits::UNITS_DEG_PER_SEC)
    {
        if (velocity < 0.0f || velocity > RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S || acceleration < 0.0f || acceleration > RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2)
        {
            params.status.status = ParamsStatus::INVALID_PARAMS;
            params.status.errorMsg = "For degree-based moves, speed must be in the range [0, " + String(RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S) + "] and acceleration must be in the range [0, " + String(RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2) + "]: speed: " + String(velocity) + ", acceleration: " + String(acceleration);
            return;
        }
        params.speed = velocity / RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S;                 // Convert to percentage of maximum velocity
        params.acceleration = acceleration / RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2; // Convert to percentage of maximum acceleration
    }
    else
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "Invalid move units";
    }
}

MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command, RobotConstants::MoveUnits moveUnits)
{
    MoveParams<RobotConstants::Robot::AXES_COUNT> params;
    params.status.status = ParamsStatus::OK;

    String paramsStr = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space

    if (paramsStr.length() == 0)
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "No parameters provided";
        return params;
    }

    int i = 0;
    bool invalidParams = false;
    int nodeCnt = 0;
    while (i < paramsStr.length() && !invalidParams && paramsStr.charAt(i) == (char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) // Parse movement parameters until we reach speed parameter (starting with 'S')
    {
        if (i + 1 >= paramsStr.length()) // It means, that the string ends with "J" without any axis identifier
        {
            invalidParams = true;
            params.status.errorMsg = "No axis identifier for the last J";
            break;
        }
        char axisIdChar = paramsStr.charAt(i + 1);
        if (axisIdChar < RobotConstants::Robot::MIN_NODE_ID || RobotConstants::Robot::MAX_NODE_ID < axisIdChar)
        {
            invalidParams = true;
            params.status.errorMsg = "Invalid axis identifier: " + String(axisIdChar);
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
                    params.status.errorMsg = "Multiple decimal points in parameter for axis " + String(axisIdChar);
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
            params.status.errorMsg = "No numeric value provided for axis " + String(axisIdChar);
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
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (nodeCnt == 0)
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "No movement parameters provided";
        return params;
    }

    if (paramsStr.substring(i, i + 2) != "SP")
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        params.status.errorMsg = "Expected speed parameter 'SP' at position " + String(i);
        return params;
    }

    stringToVelocityAndAcceleration(paramsStr.substring(i), params, moveUnits);
    return params;
}

MotorIndices stringToMotorIndices(String command)
{
    String params = command.substring(3); // Only parameters, without command and space
    MotorIndices motorIndices;
    motorIndices.status.status = ParamsStatus::OK;
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
            motorIndices.status.errorMsg = "Motor identifiers should start with '" + String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + "' followed by a letter";
            break;
        }

        char motorChar = params.charAt(i + 1);
        if (motorChar < RobotConstants::Robot::MIN_NODE_ID || motorChar > RobotConstants::Robot::MAX_NODE_ID)
        {
            isOk = false;
            motorIndices.status.errorMsg = "Invalid motor identifier: " + String(motorChar);
            break;
        }

        uint8_t nodeId = (motorChar - RobotConstants::Robot::MIN_NODE_ID) + 1;
        if (nodeId > RobotConstants::Robot::AXES_COUNT)
        {
            isOk = false;
            motorIndices.status.errorMsg = "Motor identifier out of range: " + String(motorChar);
            break;
        }

        motorIndices.nodeIds.push_back(nodeId); // Convert 'A'-'F' to 1-6
        i += 2;                                 // Skip the motor identifier
    }

    if (isOk && i != params.length())
    {
        isOk = false;
        motorIndices.status.errorMsg = "Incomplete motor identifier at end of parameters";
    }

    if (!isOk)
    {
        motorIndices.status.status = ParamsStatus::INVALID_PARAMS;
    }
    return motorIndices;
}

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, const String &command, bool isAbsoluteMove)
{
    if (params.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, params.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(command + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }

    String moveInputStr = "MAP: vel=" + String(params.speed, 3) + ", acc=" + String(params.acceleration, 3) + "; ";
    for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
    {
        moveInputStr += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + String(params.movementUnits[nodeId - 1], 3) + " ";
    }
    DBG_VERBOSE(DBG_GROUP_MOVE, moveInputStr);

    MoveController::PrepareMoveStatus movePrepareStatus = moveController.move(params, isAbsoluteMove, &command);
    if (movePrepareStatus != MoveController::PrepareMoveStatus::OK) // If something went wrong during preparation (before sending to CAN bus). 
    // If the preparation was successful, the reply will be sent later, after the move is completed or if an error occurs during the move (in MAJ_finalResult callback).
    {
        addDataToOutQueue(command + " " + MoveController::prepareMoveStatusToString(movePrepareStatus));
    }
}

void handleMotorStatus(String params)
{
    if (params != RobotConstants::Commands::MOTOR_STATUS)
    {
        DBG_VERBOSE(DBG_GROUP_COMMAND, RobotConstants::Commands::MOTOR_STATUS + " does not take any parameters");
        addDataToOutQueue(RobotConstants::Commands::MOTOR_STATUS + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    moveController.requestStatus();
}

void handlePIControl(String params)
{
    // PIC <nodeId> <paramId> <value>
    // Example: PIC JA P1 V100
    String paramsSubStr = params.substring(RobotConstants::Commands::COMMAND_LEN);
    if (!paramsSubStr.startsWith("J") || paramsSubStr.length() < 2)
    {
        DBG_ERROR(DBG_GROUP_PI, "Does not start with J followed by node identifier or not long enough");
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROL + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    
    uint8_t nodeId = static_cast<uint8_t>(paramsSubStr.charAt(1) - RobotConstants::Robot::MIN_NODE_ID) + 1;
    if (nodeId < 1 || RobotConstants::Robot::AXES_COUNT < nodeId)
    {
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROL + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    DBG_INFO(DBG_GROUP_PI, "RPI for node " + String(nodeId));
    // Find spaces
    int firstSpace = paramsSubStr.indexOf('P', 0); // Start searching after position 1
    if (firstSpace == -1){
        DBG_ERROR(DBG_GROUP_PI, "No 'P'");
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROL + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    
    int secondSpace = paramsSubStr.indexOf('V', firstSpace + 1);
    if (secondSpace == -1){
        DBG_ERROR(DBG_GROUP_PI, "No 'V'");
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROL + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    
    // Extract second parameter
    String secondParamStr = paramsSubStr.substring(firstSpace + 1, secondSpace);
    long parameterId = secondParamStr.toInt();
    DBG_INFO(DBG_GROUP_PI, "Parameter ID: " + String(parameterId));
    if (parameterId <= 0 || 4 < parameterId) // For now, we only support parameters 1-4, which correspond to P and I gains of velocity and position controllers. This can be expanded in the future if needed.
    {
        DBG_ERROR(DBG_GROUP_PI, "ParameterId invalid");
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROL + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    
    // Extract third parameter
    String thirdParamStr = paramsSubStr.substring(secondSpace + 1);
    long value = thirdParamStr.toInt();
    DBG_INFO(DBG_GROUP_PI, "third param value " + String(value));
    moveController.setPIControlParameter(nodeId, parameterId, value);
}

void handleRequestPI(MotorIndices motorIndices)
{
    if (motorIndices.status.status != ParamsStatus::OK || motorIndices.nodeIds.size() != 1) // For simplicity, for now we only support requesting PI parameters for a single axis at a time. This can be expanded in the future if needed.
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::REQUEST_PI + " " + motorIndices.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(RobotConstants::Commands::REQUEST_PI + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    moveController.startRequestPI(motorIndices.nodeIds[0]);
}


void handleZeroInitialize(MotorIndices motorIndices)
{
    if (motorIndices.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::ZERO_INITIALIZE + " " + motorIndices.status.errorMsg.value_or("no error message"));
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
    if (motorIndices.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::REQUEST_POSITION + " " + motorIndices.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(RobotConstants::Commands::REQUEST_POSITION + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    String reply = RobotConstants::Commands::REQUEST_POSITION + " " + RobotConstants::Status::OK + " ";
    for (uint8_t nodeId : motorIndices.nodeIds)
    {
        reply += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + String(moveController.axisPosition(nodeId).value_or(0)) + "; ";
    }
    addDataToOutQueue(reply);
}

void handleRequestPositionAngles(MotorIndices motorIndices)
{
    if (motorIndices.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::REQUEST_POSITION_ANGLES + " " + motorIndices.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(RobotConstants::Commands::REQUEST_POSITION_ANGLES + " " + RobotConstants::Status::INVALID_PARAMS);
        return;
    }
    String reply = RobotConstants::Commands::REQUEST_POSITION_ANGLES + " " + RobotConstants::Status::OK + " ";
    for (uint8_t nodeId : motorIndices.nodeIds)
    {
        reply += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + String(Axis::stepsToUnits(moveController.axisPosition(nodeId).value_or(0)), 3) + "; ";
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
    
    // digitalWrite(PC13, LOW);
    // delay(100);
    // digitalWrite(PC13, HIGH);
    // delay(100);
    // digitalWrite(PC13, LOW);
    // delay(100);
    // digitalWrite(PC13, HIGH);
    
    bool success = runPrepareMoveTests(isVerbose);
    if (!success)
    {
        addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_TEST + " " + RobotConstants::Status::LOGIC_ERROR);
        return;
    }
    addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_TEST + " " + RobotConstants::Status::OK);
}