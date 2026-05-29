#include <unordered_set>
#include <queue>
#include <vector>
#include <optional>
#include <cstdint>

#include "STM32_CAN.h"
#include "CanOpenController.h"
#include "CanOpen.h"
#include "Params.h"
#include "RobotConstants.h"
#include "Debug.h"
#include "Solver4AxesAlwaysVertical.h"

#include <EEPROM.h>

HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

String inData;
uint8_t bufIndex = 0;       // хранилище данных с последовательного порта
std::queue<String> outData; // очередь сообщений на отправку

// Forward declarations
SpeedAcceleration stringToSpeedAcceleration(String paramsSubStr, RobotConstants::MoveUnits moveUnits);
MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command, bool containsSpeedAcceleration = true, RobotConstants::MoveUnits moveUnits = RobotConstants::MoveUnits::UNITS_PERCENT);
MotorIndices stringToMotorIndices(String command);
MotorIndex stringToMotorIndex(String command);
MoveToleranceWrite stringToMoveToleranceWrite(String command);
PIValue stringToPIValue(String command);
MoveCartesianParams stringToMoveCartesianParams(String command, bool containsSpeedAcceleration = true, RobotConstants::MoveUnits moveUnits = RobotConstants::MoveUnits::UNITS_PERCENT);

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, const String &command, bool isAbsoluteMove);
void handleZeroInitialize(MotorIndex motorIndex);
void handlePIControllerRead(MotorIndex motorIndex);
void handlePIControllerWrite(PIValue piValue);
void handleMoveToleranceWrite(MoveToleranceWrite parsed);
void handleMoveToleranceRead(String command);
void handleJointPositions(String command, bool inSteps = false);
void handleRobotPositionCartesian(String command);
void handleRequestMotorStatus(String params);
void handleGripperWrite(String command);
void handlePinOutRead(String command);
void handlePinOutWrite(String command);
void handleReadSerialNumber(String command);
void handleWriteSerialNumber(String command);
void handleHomeJointRead(String command);
void handleMoveAbsoluteCartesian(MoveCartesianParams params, const String &command, bool isAbsoluteMove);
// stringToMoveCartesianParams
void handleMoveHomeJoint(SpeedAcceleration speedAcceleration);
void handleHomeJointWrite(MoveParams<RobotConstants::Robot::AXES_COUNT> params);
void handleMoveTimeoutRead(String command);
void handleMoveTimeoutWrite(String command);
void loadMajMoveTimeoutFromEeprom();

bool receiveCommand();
void handleCommand();
void addDataToOutQueue(String data);
void sendData();

bool isValidFloat(String str);
bool isValidInt(String str);
//String jointsToString(const float *joints);

uint32_t lastTickTime_100 = 0;
uint32_t lastTickTime_500 = 0;

void setup()
{
    // pinMode(PC13, OUTPUT);
    // digitalWrite(PC13, HIGH);

    // Serial2.setRx(PA3);
    // Serial2.setTx(PA2);

    bool canOk = canOpen.startCan(1000000);

    Serial2.begin(115200);
    while (!Serial2)
    {
    }

    if(!canOk){
        Serial2.println("COP FF");
        while(!canOk)
        {
            canOk = canOpen.startCan(1000000);
            delay(1000);
        }
    }
    else
    {
        Serial2.println("COP OK");
    }

    pinMode(GRIPPER_PIN, OUTPUT);
    for (int i = 0; i < PIN_OUT_NUM; i++)
    {
        pinMode(userPinMap[i], OUTPUT);
    }

    Serial2.println("SER OK");



    //uint8_t nodesToInvert[] = {3, 4};

    ParamsStatusStruct moveControllerInitStatus = moveController.start(&canOpen,
                                                                       RobotConstants::Robot::AXES_COUNT,
                                                                       std::make_unique<Solver4AxesAlwaysVertical<RobotConstants::Robot::AXES_COUNT>>(),
                                                                       nullptr,
                                                                       0);
    if (moveControllerInitStatus.status == ParamsStatus::INVALID_PARAMS)
    {
        Serial2.println("MVC FF " + moveControllerInitStatus.errorMsg.value_or("no error message"));
        while (1)
            ;
    }
    else
    {
        Serial2.println("MVC OK");
    }

    loadMajMoveTimeoutFromEeprom();

    inData.reserve(128); // Reserve space to avoid dynamic allocations during command reception
    Serial2.println("Setup complete!!!!");
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

    if (inData.length() < RobotConstants::Commands::COMMAND_LEN)
    {
        addDataToOutQueue(inData + " " + RobotConstants::Result::UNSUPPORTED_COMMAND);
        inData = "";
        return;
    }

    String function = inData.substring(0, RobotConstants::Commands::COMMAND_LEN);
    if (function.equals(RobotConstants::Commands::MOVE_ABSOLUTE_JOINT))
    {
        handleMove(stringToMoveParams(inData, true, RobotConstants::MoveUnits::UNITS_PERCENT), RobotConstants::Commands::MOVE_ABSOLUTE_JOINT, true);
    }
    else if (function.equals(RobotConstants::Commands::ECHO))
    {
        addDataToOutQueue(RobotConstants::Commands::ECHO + " " + RobotConstants::Result::OK + " " + inData.substring(RobotConstants::Commands::COMMAND_LEN));
    }
    else if (function.equals(RobotConstants::Commands::REQUEST_MOTOR_STATUS))
    {
        handleRequestMotorStatus(inData);
    }
    else if (function.equals(RobotConstants::Commands::ZERO_OUT_ENCODER))
    {
        handleZeroInitialize(stringToMotorIndex(inData));
    }
    else if (function.equals(RobotConstants::Commands::JOINT_POSITIONS_STEPS))
    {
        handleJointPositions(inData, true);
    }
    else if (function.equals(RobotConstants::Commands::JOINT_POSITIONS_DEGREES))
    {
        handleJointPositions(inData, false);
    }
    else if (function.equals(RobotConstants::Commands::ROBOT_POSITION_CARTESIAN))
    {
        handleRobotPositionCartesian(inData);
    }
    else if (function.equals(RobotConstants::Commands::PI_CONTROLLER_READ))
    {
        handlePIControllerRead(stringToMotorIndex(inData));
    }
    else if (function.equals(RobotConstants::Commands::PI_CONTROLLER_WRITE))
    {
        handlePIControllerWrite(stringToPIValue(inData));
    }
    else if (function.equals(RobotConstants::Commands::MOVE_TOLERANCE_WRITE))
    {
        handleMoveToleranceWrite(stringToMoveToleranceWrite(inData));
    }
    else if (function.equals(RobotConstants::Commands::MOVE_TOLERANCE_READ))
    {
        handleMoveToleranceRead(inData);
    }
    else if (function.equals(RobotConstants::Commands::GRIPPER_WRITE))
    {
        delay(1000);
        addDataToOutQueue("GRW OK");
        //handleGripperWrite(inData);
    }
    else if (function.equals(RobotConstants::Commands::GRIPPER_READ))
    {
        addDataToOutQueue(RobotConstants::Commands::GRIPPER_READ + " " + RobotConstants::Result::OK + " " + digitalRead(GRIPPER_PIN));
    }
    else if (function.equals(RobotConstants::Commands::MOVE_HOME_JOINT))
    {
        handleMoveHomeJoint(stringToSpeedAcceleration(inData, RobotConstants::MoveUnits::UNITS_PERCENT));
    }
    else if (function.equals(RobotConstants::Commands::HOME_JOINT_WRITE))
    {
        handleHomeJointWrite(stringToMoveParams(inData, false));
    }
    else if (function.equals(RobotConstants::Commands::HOME_JOINT_READ))
    {
        handleHomeJointRead(inData);
    }
    else if (function.equals(RobotConstants::Commands::MOVE_TIMEOUT_WRITE))
    {
        handleMoveTimeoutWrite(inData);
    }
    else if (function.equals(RobotConstants::Commands::MOVE_TIMEOUT_READ))
    {
        handleMoveTimeoutRead(inData);
    }
    else if (function.equals(RobotConstants::Commands::MOVE_ABSOLUTE_CARTESIAN))
    {
        delay(10000);
        addDataToOutQueue("MAC OK");
        //handleMoveAbsoluteCartesian(stringToMoveCartesianParams(inData, true, RobotConstants::MoveUnits::UNITS_PERCENT), RobotConstants::Commands::MOVE_ABSOLUTE_CARTESIAN, true);
    }
    else if (function.equals(RobotConstants::Commands::PIN_OUT_WRITE))
    {
        delay(1000);
        addDataToOutQueue("POW OK");
        //handlePinOutWrite(inData);
    }
    else if (function.equals(RobotConstants::Commands::PIN_OUT_READ))
    {
        handlePinOutRead(inData);
    }
    else if (function.equals(RobotConstants::Commands::READ_MODEL))
    {
        addDataToOutQueue(RobotConstants::Commands::READ_MODEL + " " + RobotConstants::Result::OK + " " + RobotConstants::Robot::MODEL);
    }
    else if (function.equals(RobotConstants::Commands::READ_SOFTWARE_VERSION))
    {
        addDataToOutQueue(RobotConstants::Commands::READ_SOFTWARE_VERSION + " " + RobotConstants::Result::OK + " " + RobotConstants::Robot::SOFTWARE_VERSION);
    }
    else if (function.equals(RobotConstants::Commands::READ_SERIAL_NUMBER))
    {
        handleReadSerialNumber(inData);
    }
    else if (function.equals(RobotConstants::Commands::WRITE_SERIAL_NUMBER))
    {
        handleWriteSerialNumber(inData);
    }
    else
    {
        //Serial2.println("RRR UC");
        addDataToOutQueue(function + " " + RobotConstants::Result::UNSUPPORTED_COMMAND);
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

    Serial2.println(data);
}

bool isValidFloat(String str)
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

bool isValidInt(String str)
{
    str.trim();
    if (str.length() == 0)
        return false;

    int start = (str[0] == '-' || str[0] == '+') ? 1 : 0;
    if (start >= str.length())
        return false;

    for (int i = start; i < str.length(); i++)
    {
        if (!isDigit(str[i]))
            return false;
    }
    return true;
}

SpeedAcceleration stringToSpeedAcceleration(String paramsSubStr, RobotConstants::MoveUnits moveUnits)
{
    SpeedAcceleration result;
    result.status.status = ParamsStatus::OK;

    int indexSP = paramsSubStr.indexOf("SP");
    int indexAC = paramsSubStr.indexOf("AC");
    if (indexSP == -1 || indexAC == -1)
    {
        result.status.status = ParamsStatus::INCORRECT_COMMAND;
        return result;
    }

    String velocityStr = paramsSubStr.substring(indexSP + 2, indexAC);
    if (!isValidFloat(velocityStr))
    {
        result.status.status = ParamsStatus::INVALID_PARAMS;
        result.status.errorMsg = "Invalid speed value: " + velocityStr;
        return result;
    }

    float velocity = velocityStr.toFloat();

    String accelerationStr = paramsSubStr.substring(indexAC + 2);
    if (!isValidFloat(accelerationStr))
    {
        result.status.status = ParamsStatus::INVALID_PARAMS;
        result.status.errorMsg = "Invalid acceleration value: " + accelerationStr;
        return result;
    }

    float acceleration = accelerationStr.toFloat();

    if (moveUnits == RobotConstants::MoveUnits::UNITS_PERCENT)
    {
        if (velocity < 0.0f || velocity > 100.0f || acceleration < 0.0f || acceleration > 100.0f)
        {
            result.status.status = ParamsStatus::INVALID_PARAMS;
            result.status.errorMsg = "For percentage-based moves, speed and acceleration must be in the range [0, 100]: speed: " + String(velocity) + ", acceleration: " + String(acceleration);
            return result;
        }
        result.speed = velocity / 100.0;
        result.acceleration = acceleration / 100.0;
    }
    else if (moveUnits == RobotConstants::MoveUnits::UNITS_DEG_PER_SEC)
    {
        if (velocity < 0.0f || velocity > RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S || acceleration < 0.0f || acceleration > RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2)
        {
            result.status.status = ParamsStatus::INVALID_PARAMS;
            result.status.errorMsg = "For degree-based moves, speed must be in the range [0, " + String(RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S) + "] and acceleration must be in the range [0, " + String(RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2) + "]: speed: " + String(velocity) + ", acceleration: " + String(acceleration);
            return result;
        }
        result.speed = velocity / RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S;                 // Convert to percentage of maximum velocity
        result.acceleration = acceleration / RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2; // Convert to percentage of maximum acceleration
    }
    else
    {
        result.status.status = ParamsStatus::INCORRECT_COMMAND;
        result.status.errorMsg = "Invalid move units";
    }
    return result;
}

MoveParams<RobotConstants::Robot::AXES_COUNT> stringToMoveParams(String command, bool containsSpeedAcceleration, RobotConstants::MoveUnits moveUnits)
{
    MoveParams<RobotConstants::Robot::AXES_COUNT> params;
    params.status.status = ParamsStatus::OK;

    String paramsStr = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space

    if (paramsStr.length() == 0)
    {
        params.status.status = ParamsStatus::INCORRECT_COMMAND;
        params.status.errorMsg = "No parameters provided";
        return params;
    }

    int i = 0;
    bool invalidParams = false;
    bool incorrectCommand = false;
    int nodeCnt = 0;
    while (i < paramsStr.length() && !invalidParams && !incorrectCommand && paramsStr.charAt(i) == (char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) // Parse movement parameters until we reach speed parameter (starting with 'S')
    {
        if (i + 1 >= paramsStr.length()) // It means, that the string ends with "J" without any axis identifier
        {
            incorrectCommand = true;
            params.status.errorMsg = "No axis identifier for the last J";
            break;
        }
        char axisIdChar = paramsStr.charAt(i + 1);
        if (axisIdChar < RobotConstants::Robot::MIN_NODE_ID || RobotConstants::Robot::MAX_NODE_ID < axisIdChar)
        {
            incorrectCommand = true;
            params.status.errorMsg = "Invalid axis identifier: " + String(axisIdChar);
            break;
        }

        int j = i + 2;
        bool decimalPointFound = false;
        if (paramsStr.charAt(j) == '-' || paramsStr.charAt(j) == '+')
            j++; // Skip sign if present

        while (j < paramsStr.length() && !invalidParams && !incorrectCommand)
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

        if (invalidParams || incorrectCommand)
        {
            break;
        }

        float movementUnits = paramsStr.substring(i + 2, j).toFloat();
        params.angles.angles[axisIdChar - RobotConstants::Robot::MIN_NODE_ID] = movementUnits; // Convert 'A'-'F' to 0-5 and store movement units
        nodeCnt++;
        i = j;
    }

    if (invalidParams)
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (incorrectCommand)
    {
        params.status.status = ParamsStatus::INCORRECT_COMMAND;
        return params;
    }

    if (nodeCnt == 0)
    {
        params.status.status = ParamsStatus::INCORRECT_COMMAND;
        params.status.errorMsg = "No movement parameters provided";
        return params;
    }

    for (uint8_t i = 0; i < RobotConstants::Robot::AXES_COUNT; ++i)
    {
        if (params.angles.angles[i] < RobotConstants::Axis::DEFAULT_MIN_LIMITS[i] || RobotConstants::Axis::DEFAULT_MAX_LIMITS[i] < params.angles.angles[i])
        {
            params.status.status = ParamsStatus::OUT_OF_LIMITS;
            return params;
        }
    }

    if (containsSpeedAcceleration)
    {
        if (paramsStr.substring(i, i + 2) != "SP")
        {
            params.status.status = ParamsStatus::INCORRECT_COMMAND;
            params.status.errorMsg = "Expected speed parameter 'SP' at position " + String(i);
            return params;
        }
        SpeedAcceleration speedAcceleration = stringToSpeedAcceleration(paramsStr.substring(i), moveUnits);
        params.status.status = speedAcceleration.status.status;
        params.move = speedAcceleration;
    }
    return params;
}

MotorIndices stringToMotorIndices(String command)
{
    String params = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space
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

MotorIndex stringToMotorIndex(String command)
{
    String params = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space
    MotorIndex motorIndex;
    motorIndex.status.status = ParamsStatus::OK;

    int anStart = params.indexOf("AN");
    if (anStart == -1)
    {
        motorIndex.status.status = ParamsStatus::INCORRECT_COMMAND;
        return motorIndex;
    }

    String motorIndexStr = params.substring(anStart + 2);
    if (!isValidInt(motorIndexStr))
    {
        motorIndex.status.status = ParamsStatus::INVALID_PARAMS;
        return motorIndex;
    }

    int nodeIdInt = motorIndexStr.toInt();
    if (nodeIdInt < 1 || nodeIdInt > RobotConstants::Robot::AXES_COUNT)
    {
        motorIndex.status.status = ParamsStatus::INVALID_PARAMS;
        return motorIndex;
    }

    motorIndex.nodeId = static_cast<uint8_t>(nodeIdInt);
    return motorIndex;
}

MoveToleranceWrite stringToMoveToleranceWrite(String command)
{
    MoveToleranceWrite out;
    out.status.status = ParamsStatus::OK;

    const int wvIndex = command.indexOf("WV");
    if (wvIndex == -1)
    {
        out.status.status = ParamsStatus::INCORRECT_COMMAND;
        return out;
    }

    const MotorIndex motorIndex = stringToMotorIndex(command.substring(0, wvIndex));
    if (motorIndex.status.status != ParamsStatus::OK)
    {
        out.status = motorIndex.status;
        return out;
    }
    out.nodeId = motorIndex.nodeId;

    const String tolStr = command.substring(wvIndex + 2);
    if (!isValidInt(tolStr))
    {
        out.status.status = ParamsStatus::INVALID_PARAMS;
        return out;
    }

    const long tol = tolStr.toInt();
    if (tol < 1L || tol > 100000)
    {
        out.status.status = ParamsStatus::INVALID_PARAMS;
        return out;
    }
    out.toleranceSteps = static_cast<uint32_t>(tol);
    return out;
}

PIValue stringToPIValue(String command)
{
    PIValue piValue;
    piValue.status.status = ParamsStatus::OK;
    int VPIndex = command.indexOf("VP");
    if (VPIndex == -1)
    {
        piValue.status.status = ParamsStatus::INCORRECT_COMMAND;
        return piValue;
    }
    MotorIndex motorIndex = stringToMotorIndex(command.substring(0, VPIndex));
    if (motorIndex.status.status != ParamsStatus::OK)
    {
        piValue.status.status = motorIndex.status.status;
        return piValue;
    }
    piValue.nodeId = motorIndex.nodeId;

    int VIIndex = command.indexOf("VI");
    int PPIndex = command.indexOf("PP");
    int FFIndex = command.indexOf("FF");

    if (VIIndex == -1 || PPIndex == -1 || FFIndex == -1)
    {
        piValue.status.status = ParamsStatus::INCORRECT_COMMAND;
        return piValue;
    }

    if (!isValidInt(command.substring(VPIndex + 2, VIIndex)))
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(VPIndex + 2, VIIndex));
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
        return piValue;
    }
    piValue.vp = command.substring(VPIndex + 2, VIIndex).toInt();
    if (piValue.vp < 0 || piValue.vp > 10000)
    {
        Serial2.println("THis: " + String(piValue.vp));
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(VPIndex + 2, VIIndex) + " bad limit");
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
        return piValue;
    }

    if (!isValidInt(command.substring(VIIndex + 2, PPIndex)))
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(VIIndex + 2, PPIndex));
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
    }
    piValue.vi = command.substring(VIIndex + 2, PPIndex).toInt();
    if (piValue.vi < 2 || piValue.vi > 2000)
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(VIIndex + 2, PPIndex) + " bad limit");
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
        return piValue;
    }

    if (!isValidInt(command.substring(PPIndex + 2, FFIndex)))
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(PPIndex + 2, FFIndex));
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
    }
    piValue.pp = command.substring(PPIndex + 2, FFIndex).toInt();
    if (piValue.pp < 60 || piValue.pp > 30000)
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(PPIndex + 2, FFIndex) + " bad limit");
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
        return piValue;
    }

    if (!isValidInt(command.substring(FFIndex + 2)))
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(FFIndex + 2));
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
    }
    piValue.ff = command.substring(FFIndex + 2).toInt();
    if (piValue.ff < 0 || piValue.ff > 3000)
    {
        addDataToOutQueue("PIW INVALID PARAMS: " + command.substring(FFIndex + 2) + " bad limit");
        piValue.status.status = ParamsStatus::INVALID_PARAMS;
        return piValue;
    }

    return piValue;
}

MoveCartesianParams stringToMoveCartesianParams(String command, bool containsSpeedAcceleration, RobotConstants::MoveUnits moveUnits)
{
    MoveCartesianParams params;
    params.status.status = ParamsStatus::OK;

    String paramsStr = command.substring(RobotConstants::Commands::COMMAND_LEN); // Only parameters, without command and space

    int PXIndex = paramsStr.indexOf("PX");
    int PYIndex = paramsStr.indexOf("PY");
    int PZIndex = paramsStr.indexOf("PZ");
    int ORIndex = paramsStr.indexOf("OR");
    int OPIndex = paramsStr.indexOf("OP");
    int OWIndex = paramsStr.indexOf("OW");

    if (PXIndex == -1 || PYIndex == -1 || PZIndex == -1 || ORIndex == -1 || OPIndex == -1 || OWIndex == -1)
    {
        params.status.status = ParamsStatus::INCORRECT_COMMAND;
        return params;
    }

    if (!isValidFloat(paramsStr.substring(PXIndex + 2, PYIndex)))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (!isValidFloat(paramsStr.substring(PYIndex + 2, PZIndex)))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (!isValidFloat(paramsStr.substring(PZIndex + 2, ORIndex)))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (!isValidFloat(paramsStr.substring(ORIndex + 2, OPIndex)))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (!isValidFloat(paramsStr.substring(OPIndex + 2, OWIndex)))
    {
        params.status.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    if (containsSpeedAcceleration)
    {
        int SPIndex = paramsStr.indexOf("SP");
        if (!isValidFloat(paramsStr.substring(OWIndex + 2, SPIndex)))
        {
            params.status.status = ParamsStatus::INVALID_PARAMS;
            return params;
        }
        params.position.yaw = paramsStr.substring(OWIndex + 2, SPIndex).toFloat();
    }
    else
    {
        if (!isValidFloat(paramsStr.substring(OWIndex + 2)))
        {
            params.status.status = ParamsStatus::INVALID_PARAMS;
            return params;
        }
        params.position.yaw = paramsStr.substring(OWIndex + 2).toFloat();
    }

    params.position.x = paramsStr.substring(PXIndex + 2, PYIndex).toFloat();
    params.position.y = paramsStr.substring(PYIndex + 2, PZIndex).toFloat();
    params.position.z = paramsStr.substring(PZIndex + 2, ORIndex).toFloat();
    params.position.roll = paramsStr.substring(ORIndex + 2, OPIndex).toFloat();
    params.position.pitch = paramsStr.substring(OPIndex + 2, OWIndex).toFloat();

    if (containsSpeedAcceleration)
    {
        SpeedAcceleration speedAcceleration = stringToSpeedAcceleration(paramsStr, moveUnits);
        params.status.status = speedAcceleration.status.status;
        params.move = speedAcceleration;
    }
    return params;
}

void handleMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params, const String &command, bool isAbsoluteMove)
{
    if (params.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, params.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(command + " " + params.status.toStr());
        return;
    }

    String moveInputStr = "MAP: vel=" + String(params.move.speed, 3) + ", acc=" + String(params.move.acceleration, 3) + "; ";
    for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
    {
        moveInputStr += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + String(params.angles.angles[nodeId - 1], 3) + " ";
    }
    DBG_VERBOSE(DBG_GROUP_MOVE, moveInputStr);

    MoveController::PrepareMoveStatus movePrepareStatus = moveController.move(params, isAbsoluteMove, &command);
    if (movePrepareStatus == MoveController::PrepareMoveStatus::NO_EFFECTIVE_MOTION)
    {
        addDataToOutQueue(command + " " + MoveController::prepareMoveStatusToString(MoveController::PrepareMoveStatus::OK) + " " + moveController.jointPositions());
    }
    else if (movePrepareStatus != MoveController::PrepareMoveStatus::OK) // If something went wrong during preparation (before sending to CAN bus).
    // If the preparation was successful, the reply will be sent later, after the move is completed or if an error occurs during the move (in MAJ_finalResult callback).
    {
        addDataToOutQueue(command + " " + MoveController::prepareMoveStatusToString(movePrepareStatus));
    }
}

void handleRequestMotorStatus(String params)
{
    if (params != RobotConstants::Commands::REQUEST_MOTOR_STATUS)
    {
        DBG_VERBOSE(DBG_GROUP_COMMAND, RobotConstants::Commands::REQUEST_MOTOR_STATUS + " does not take any parameters");
        addDataToOutQueue(RobotConstants::Commands::REQUEST_MOTOR_STATUS + " " + RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }
    addDataToOutQueue(RobotConstants::Commands::REQUEST_MOTOR_STATUS + " " + RobotConstants::Result::OK + " " + moveController.requestStatus());
}

void handleGripperWrite(String command)
{
    if (command.length() != RobotConstants::Commands::COMMAND_LEN + 1)
    {
        addDataToOutQueue(RobotConstants::Commands::GRIPPER_WRITE + " " + RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }
    if (command[3] == '0')
    {
        //digitalWrite(GRIPPER_PIN, LOW);
        //digitalWrite(PB6, LOW);
        digitalWrite(PB7, LOW);
        digitalWrite(PC13, LOW);
        digitalWrite(PB5, LOW);
        addDataToOutQueue(RobotConstants::Commands::GRIPPER_WRITE + " " + RobotConstants::Result::OK + " 0");
        return;
    }
    else if (command[3] == '1')
    {
        //digitalWrite(GRIPPER_PIN, HIGH);
        //digitalWrite(PB6, HIGH);
        digitalWrite(PB7, HIGH);
        digitalWrite(PC13, HIGH);
        digitalWrite(PB5, HIGH);
        addDataToOutQueue(RobotConstants::Commands::GRIPPER_WRITE + " " + RobotConstants::Result::OK + " 1");
        return;
    }
    else
    {
        addDataToOutQueue(RobotConstants::Commands::GRIPPER_WRITE + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
}

void handlePinOutRead(String command)
{
    if (command.length() != RobotConstants::Commands::COMMAND_LEN + 1)
    {
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_READ + " " + RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }
    int pinNum = command.charAt(3) - '0';
    if (pinNum >= 1 && pinNum <= PIN_OUT_NUM)
    {
        // Map to actual pin and set value
        uint32_t actualPin = userPinMap[pinNum - 1]; // -1 because array is 0-indexed
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_READ + " " + RobotConstants::Result::OK + " OP" + command.charAt(3) + " WV" + digitalRead(actualPin));
        return;
    }
    addDataToOutQueue(RobotConstants::Commands::PIN_OUT_READ + " " + RobotConstants::Result::INVALID_PARAMS);
}

void handlePinOutWrite(String command)
{
    int POIndex = command.indexOf("OP");
    int WVIndex = command.indexOf("WV");
    if (POIndex == -1 || WVIndex == -1)
    {
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }
    String pinOutStr = command.substring(POIndex + 2, WVIndex);
    if (!isValidInt(pinOutStr))
    {
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
    int pinNum = pinOutStr.toInt();
    if (pinNum < 1 || PIN_OUT_NUM < pinNum)
    {
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
    uint32_t actualPin = userPinMap[pinNum - 1];

    String valueStr = command.substring(WVIndex + 2);
    if (!isValidInt(valueStr))
    {
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
    int value = valueStr.toInt();
    if (value < 0 || 1 < value)
    {
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
    else if (value == 0)
    {
        digitalWrite(actualPin, LOW);
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::OK + " OP" + pinOutStr + " WV0");
    }
    else
    {
        digitalWrite(actualPin, HIGH);
        addDataToOutQueue(RobotConstants::Commands::PIN_OUT_WRITE + " " + RobotConstants::Result::OK + " OP" + pinOutStr + " WV1");
    }
}

void handleReadSerialNumber(String command)
{
    if (command != RobotConstants::Commands::READ_SERIAL_NUMBER)
    {
        addDataToOutQueue(RobotConstants::Commands::READ_SERIAL_NUMBER + " " + RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }

    RobotConstants::Eeprom::SerialNumber data;
    EEPROM.get(RobotConstants::Eeprom::SERIAL_NUMBER_ADDR, data);
    if (data.magic != RobotConstants::Eeprom::SERIAL_MAGIC)
    {
        addDataToOutQueue(RobotConstants::Commands::READ_SERIAL_NUMBER + " " + RobotConstants::Result::NO_DATA);
    }
    else
    {
        data.serial[sizeof(data.serial) - 1] = '\0';
        addDataToOutQueue(RobotConstants::Commands::READ_SERIAL_NUMBER + " " + RobotConstants::Result::OK + " " + String(data.serial));
    }
}

void handleWriteSerialNumber(String command)
{
    RobotConstants::Eeprom::SerialNumber data;
    EEPROM.get(RobotConstants::Eeprom::SERIAL_NUMBER_ADDR, data);

    if (data.magic == RobotConstants::Eeprom::SERIAL_MAGIC)
    {
        addDataToOutQueue(RobotConstants::Commands::WRITE_SERIAL_NUMBER + " " + RobotConstants::Result::UNSUPPORTED_COMMAND);
        return;
    }

    String serialNumber = command.substring(RobotConstants::Commands::COMMAND_LEN);
    if (serialNumber.length() > RobotConstants::Robot::SERIAL_NUMBER_MAX_LENGTH)
    {
        addDataToOutQueue(RobotConstants::Commands::WRITE_SERIAL_NUMBER + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
    data.magic = RobotConstants::Eeprom::SERIAL_MAGIC;
    strncpy(data.serial, serialNumber.c_str(), sizeof(data.serial) - 1);
    data.serial[sizeof(data.serial) - 1] = '\0';

    EEPROM.put(RobotConstants::Eeprom::SERIAL_NUMBER_ADDR, data);

    EEPROM.get(RobotConstants::Eeprom::SERIAL_NUMBER_ADDR, data);
    addDataToOutQueue(RobotConstants::Commands::WRITE_SERIAL_NUMBER + " " + RobotConstants::Result::OK + " " + String(data.serial));
}

void loadMajMoveTimeoutFromEeprom()
{
    RobotConstants::Eeprom::MoveTimeout data;
    EEPROM.get(RobotConstants::Eeprom::MOVE_TIMEOUT_ADDR, data);
    if (data.magic != RobotConstants::Eeprom::MOVE_TIMEOUT_MAGIC)
    {
        return;
    }
    if (data.timeoutSec < RobotConstants::Control::MIN_MAJ_MOVE_TIMEOUT_SEC ||
        data.timeoutSec > RobotConstants::Control::MAX_MAJ_MOVE_TIMEOUT_SEC)
    {
        return;
    }
    moveController.setMajMoveTimeoutMs(data.timeoutSec * 1000u);
}

void handleMoveTimeoutRead(String command)
{
    if (command != RobotConstants::Commands::MOVE_TIMEOUT_READ)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_READ + " " +
                          RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }

    RobotConstants::Eeprom::MoveTimeout data;
    EEPROM.get(RobotConstants::Eeprom::MOVE_TIMEOUT_ADDR, data);
    if (data.magic != RobotConstants::Eeprom::MOVE_TIMEOUT_MAGIC)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_READ + " " + RobotConstants::Result::NO_DATA);
        return;
    }

    addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_READ + " " + RobotConstants::Result::OK + " " +
                      String(data.timeoutSec));
}

void handleMoveTimeoutWrite(String command)
{
    if (command.length() <= RobotConstants::Commands::COMMAND_LEN)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_WRITE + " " +
                          RobotConstants::Result::INVALID_PARAMS);
        return;
    }

    const String secStr = command.substring(RobotConstants::Commands::COMMAND_LEN);
    if (!isValidInt(secStr))
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_WRITE + " " +
                          RobotConstants::Result::INVALID_PARAMS);
        return;
    }

    const long secSigned = secStr.toInt();
    if (secSigned < static_cast<long>(RobotConstants::Control::MIN_MAJ_MOVE_TIMEOUT_SEC) ||
        secSigned > static_cast<long>(RobotConstants::Control::MAX_MAJ_MOVE_TIMEOUT_SEC))
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_WRITE + " " +
                          RobotConstants::Result::INVALID_PARAMS);
        return;
    }

    const uint32_t sec = static_cast<uint32_t>(secSigned);

    RobotConstants::Eeprom::MoveTimeout data{};
    data.magic = RobotConstants::Eeprom::MOVE_TIMEOUT_MAGIC;
    data.timeoutSec = sec;

    EEPROM.put(RobotConstants::Eeprom::MOVE_TIMEOUT_ADDR, data);
    EEPROM.get(RobotConstants::Eeprom::MOVE_TIMEOUT_ADDR, data);
    if (data.magic != RobotConstants::Eeprom::MOVE_TIMEOUT_MAGIC || data.timeoutSec != sec)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_WRITE + " " + RobotConstants::Result::FAIL);
        return;
    }

    moveController.setMajMoveTimeoutMs(sec * 1000u);
    addDataToOutQueue(RobotConstants::Commands::MOVE_TIMEOUT_WRITE + " " + RobotConstants::Result::OK + " " +
                      String(sec));
}

void handleHomeJointRead(String command)
{
    if (inData != RobotConstants::Commands::HOME_JOINT_READ)
    {
        addDataToOutQueue(RobotConstants::Commands::HOME_JOINT_READ + " " + RobotConstants::Result::INCORRECT_COMMAND);
        return;
    }

    RobotConstants::Eeprom::Home data;
    EEPROM.get(RobotConstants::Eeprom::HOME_JOINT_ADDR, data);
    if (data.magic != RobotConstants::Eeprom::HOME_JOINT_MAGIC)
    {
        addDataToOutQueue(RobotConstants::Commands::HOME_JOINT_READ + " " + RobotConstants::Result::NO_DATA);
    }
    else
    {
        addDataToOutQueue(
            RobotConstants::Commands::HOME_JOINT_READ + " " +
            RobotConstants::Result::OK + " " +
            MoveController::jointsToString(data.joints));
    }
}

void handleMoveAbsoluteCartesian(MoveCartesianParams params, const String &command, bool isAbsoluteMove)
{
    if (params.status.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_ABSOLUTE_CARTESIAN + " " + params.status.toStr());
        return;
    }

    // MoveController::PrepareMoveStatus movePrepareStatus = moveController.move(params, isAbsoluteMove, &command);
    MoveController::PrepareMoveStatus movePrepareStatus = moveController.moveCartesian(params, isAbsoluteMove, &command);

    if (movePrepareStatus == MoveController::PrepareMoveStatus::NO_EFFECTIVE_MOTION)
    {
        addDataToOutQueue(command + " " + MoveController::prepareMoveStatusToString(MoveController::PrepareMoveStatus::OK) + " " + moveController.jointPositions());
    }
    else if (movePrepareStatus != MoveController::PrepareMoveStatus::OK) // If something went wrong during preparation (before sending to CAN bus).
    // If the preparation was successful, the reply will be sent later, after the move is completed or if an error occurs during the move (in MAJ_finalResult callback).
    {
        addDataToOutQueue(command + " " + MoveController::prepareMoveStatusToString(movePrepareStatus));
    }
}

void handleMoveHomeJoint(SpeedAcceleration speedAcceleration)
{
    if (speedAcceleration.status.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_HOME_JOINT + " " + speedAcceleration.status.toStr());
        return;
    }

    RobotConstants::Eeprom::Home data;
    EEPROM.get(RobotConstants::Eeprom::HOME_JOINT_ADDR, data);
    if (data.magic != RobotConstants::Eeprom::HOME_JOINT_MAGIC)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_HOME_JOINT + " " + RobotConstants::Result::NO_DATA);
        return;
    }

    String jointsString = MoveController::jointsToString(data.joints);

    MoveParams<RobotConstants::Robot::AXES_COUNT> moveParams;
    moveParams.status.status = ParamsStatus::OK;
    for (uint8_t i = 0; i < RobotConstants::Robot::AXES_COUNT; ++i)
    {
        moveParams.angles.angles[i] = data.joints[i];
    }
    moveParams.move = speedAcceleration;

    handleMove(moveParams, RobotConstants::Commands::MOVE_HOME_JOINT, true);
}

void handleHomeJointWrite(MoveParams<RobotConstants::Robot::AXES_COUNT> params)
{
    if (params.status.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::HOME_JOINT_WRITE + " " + params.status.toStr());
        return;
    }

    RobotConstants::Eeprom::Home data;
    data.magic = RobotConstants::Eeprom::HOME_JOINT_MAGIC;

    for (uint8_t i = 0; i < RobotConstants::Robot::AXES_COUNT; i++)
    {
        data.joints[i] = params.angles.angles[i];
    }
    for (uint8_t i = RobotConstants::Robot::AXES_COUNT; i < 6; ++i)
    {
        data.joints[i] = 0;
    }

    EEPROM.put(RobotConstants::Eeprom::HOME_JOINT_ADDR, data);

    EEPROM.get(RobotConstants::Eeprom::HOME_JOINT_ADDR, data);
    addDataToOutQueue(
        RobotConstants::Commands::HOME_JOINT_WRITE + " " +
        RobotConstants::Result::OK + " " +
        MoveController::jointsToString(data.joints));
}

void handleRequestPI(MotorIndices motorIndices)
{
    if (motorIndices.status.status != ParamsStatus::OK || motorIndices.nodeIds.size() != 1) // For simplicity, for now we only support requesting PI parameters for a single axis at a time. This can be expanded in the future if needed.
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::PI_CONTROLLER_READ + " " + motorIndices.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROLLER_READ + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }
    moveController.startRequestPI(motorIndices.nodeIds[0]);
}

void handleZeroInitialize(MotorIndex motorIndex)
{
    if (motorIndex.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::ZERO_OUT_ENCODER + " " + motorIndex.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(RobotConstants::Commands::ZERO_OUT_ENCODER + " " + motorIndex.status.toStr());
        return;
    }
    // if (motorIndices.nodeIds.size() == RobotConstants::Robot::AXES_COUNT)
    // {
    //     DBG_VERBOSE(DBG_GROUP_ZOE, RobotConstants::Commands::ZERO_OUT_ENCODER + " Starting Zero Initialization for all nodes");
    //     moveController.startZeroInitializationAllAxes();
    // }

    DBG_VERBOSE(DBG_GROUP_ZOE, RobotConstants::Commands::ZERO_OUT_ENCODER + " Starting Zero Initialization for node " + String(motorIndex.nodeId));
    moveController.startZeroInitializationSingleAxis(motorIndex.nodeId);
}

void handlePIControllerRead(MotorIndex motorIndex)
{
    if (motorIndex.status.status != ParamsStatus::OK)
    {
        DBG_WARN(DBG_GROUP_COMMAND, RobotConstants::Commands::PI_CONTROLLER_READ + " " + motorIndex.status.errorMsg.value_or("no error message"));
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROLLER_READ + " " + motorIndex.status.toStr());
        return;
    }

    moveController.startRequestPI(motorIndex.nodeId);
}

void handlePIControllerWrite(PIValue piValue)
{
    if (piValue.status.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::PI_CONTROLLER_WRITE + " BAD " + piValue.status.toStr());
        return;
    }

    moveController.setPIController(piValue.nodeId, piValue);
}

void handleMoveToleranceWrite(MoveToleranceWrite parsed)
{
    if (parsed.status.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_WRITE + " " + parsed.status.toStr());
        return;
    }

    if (moveController.isMoveInProgress())
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_WRITE + " " + RobotConstants::Result::OPERATION_FORBIDDEN);
        return;
    }

    if (!moveController.setMajMoveToleranceSteps(parsed.nodeId, parsed.toleranceSteps))
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_WRITE + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }

    addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_WRITE + " " + RobotConstants::Result::OK +
                      " AN" + String(parsed.nodeId) + " WV" + String(parsed.toleranceSteps));
}

void handleMoveToleranceRead(String command)
{
    MotorIndex motorIndex = stringToMotorIndex(command);
    if (motorIndex.status.status != ParamsStatus::OK)
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_READ + " " + motorIndex.status.toStr());
        return;
    }

    std::optional<uint32_t> tol = moveController.getMajMoveToleranceSteps(motorIndex.nodeId);
    if (!tol.has_value())
    {
        addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_READ + " " + RobotConstants::Result::INVALID_PARAMS);
        return;
    }

    addDataToOutQueue(RobotConstants::Commands::MOVE_TOLERANCE_READ + " " + RobotConstants::Result::OK +
                      " AN" + String(motorIndex.nodeId) + " WV" + String(tol.value()));
}

void handleJointPositions(String command, bool inSteps)
{
    String commandStr = command.substring(0, 3);
    if (command.length() > RobotConstants::Commands::COMMAND_LEN)
    {
        addDataToOutQueue(commandStr + " " + RobotConstants::Result::INCORRECT_COMMAND);
    }

    String positions = moveController.jointPositions(inSteps);
    if (positions == "")
    {
        addDataToOutQueue(commandStr + " " + RobotConstants::Result::NOT_INITIALIZED);
    }
    else
    {
        addDataToOutQueue(commandStr + " " + RobotConstants::Result::OK + " " + positions);
    }
}

void handleRobotPositionCartesian(String command)
{
    if (command != RobotConstants::Commands::ROBOT_POSITION_CARTESIAN)
    {
        addDataToOutQueue(RobotConstants::Commands::ROBOT_POSITION_CARTESIAN + " " + RobotConstants::Result::INCORRECT_COMMAND);
    }
    addDataToOutQueue(RobotConstants::Commands::ROBOT_POSITION_CARTESIAN + " " + RobotConstants::Result::OK + " " + moveController.cartesianPosition());
}