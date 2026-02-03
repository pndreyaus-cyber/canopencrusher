#include <unordered_set>

#include "STM32_CAN.h"
#include "CanOpenController.h"
#include "CanOpen.h"
#include "Params.h"
#include "RobotConstants.h"

const double MAX_SPEED = 360;
const double MAX_ACCELERATION = 7864.20;

HardwareSerial Serial2(PA3, PA2);

CanOpen canOpen;
MoveController moveController;

uint8_t buf[8];

String inData;
uint8_t bufIndex = 0;        // хранилище данных с последовательного порта
std::vector<String> outData; // очередь сообщений на отправку

// Forward declarations
MoveParams<RobotConstants::Robot::AXIS_COUNT> stringToMoveParams(String command);
PositionParams stringToPositionParams(String command);
ZEIParams stringToZEIParams(String command);
int stringToNodeId(String command);
bool handleMove(MoveParams<RobotConstants::Robot::AXIS_COUNT> params, bool isAbsoluteMove);
bool handleSetCurrentPositionInSteps(PositionParams params);
bool handleSetCurrentPositionInUnits(PositionParams params);
bool handleZeroInitialize(ZEIParams params);
bool handleRequestPosition(int parsedId);

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
        inData = "";
        addDataToOutQueue("INVALID COMMAND: TOO SHORT");
        return;
    }

    String function = inData.substring(0, 3);

    if (function.equals(RobotConstants::COMMANDS::MOVE_ABSOLUTE))
    {
        if (handleMove(stringToMoveParams(inData), true))
        {
            addDataToOutQueue("MAJ COMMAND COMPLETED");
        }
        else
        {
            addDataToOutQueue("MAJ COMMAND FAILED");
        }
    }

    else if (function.equals(RobotConstants::COMMANDS::MOVE_RELATIVE))
    {
        if (handleMove(stringToMoveParams(inData), false))
        {
            addDataToOutQueue("MRJ COMMAND COMPLETED");
        }
        else
        {
            addDataToOutQueue("MRJ COMMAND FAILED");
        }
    }
    else if (function.equals(RobotConstants::COMMANDS::ECHO))
    {
        addDataToOutQueue(inData.substring(4));
    }
    else if (function.equals(RobotConstants::COMMANDS::SET_CURRENT_POSITION_IN_STEPS))
    {
        if (handleSetCurrentPositionInSteps(stringToPositionParams(inData)))
        {
            addDataToOutQueue("SCS COMMAND COMPLETED");
        }
        else
        {
            addDataToOutQueue("SCS COMMAND FAILED");
        }
    }
    else if (function.equals(RobotConstants::COMMANDS::SET_CURRENT_POSITION_IN_UNITS))
    {
        if (handleSetCurrentPositionInUnits(stringToPositionParams(inData)))
        {
            addDataToOutQueue("SCU COMMAND COMPLETED");
        }
        else
        {
            addDataToOutQueue("SCU COMMAND FAILED");
        }
    }
    else if (function.equals(RobotConstants::COMMANDS::ZERO_INITIALIZE))
    {
        handleZeroInitialize(stringToZEIParams(inData));
    }
    else if (function.equals(RobotConstants::COMMANDS::REQUEST_POSITION))
    {
        if (handleRequestPosition(stringToNodeId(inData)))
        {
            addDataToOutQueue("RPP COMMAND COMPLETED");
        }
        else
        {
            addDataToOutQueue("RPP COMMAND FAILED");
        }
    }
    else
    {
        addDataToOutQueue("INVALID COMMAND");
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

PositionParams stringToPositionParams(String command)
{
    PositionParams params;

    int positionIndex = command.indexOf("POS");
    int idIndex = command.indexOf("ID");

    if ((positionIndex == -1) || (idIndex == -1))
    {
        params.status = ParamsStatus::INCORRECT_COMMAND;
        return params;
    }

    params.currentPosition = command.substring(positionIndex + 3, idIndex).toFloat();
    params.nodeId = command.substring(idIndex + 2).toInt();

    if ((params.nodeId <= 0) || (params.nodeId > RobotConstants::Robot::AXIS_COUNT))
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        return params;
    }

    params.status = ParamsStatus::OK;
    return params;
}

// Right now we support either ZEI for all nodes or only for 1 node specified by its ID.
ZEIParams stringToZEIParams(String command)
{
    if (command.equals(RobotConstants::COMMANDS::ZERO_INITIALIZE))
    {
        ZEIParams params;
        params.status = ParamsStatus::OK;
        params.forAllNodes = true;
        params.errorMsg = "";
        return params;
    }

    ZEIParams params;
    params.status = ParamsStatus::INVALID_PARAMS;
    params.forAllNodes = false;
    params.errorMsg = "INVALID PARAMS FOR ZEI";

    auto fail = [&](const String &msg)
    {
        params.status = ParamsStatus::INVALID_PARAMS;
        params.forAllNodes = false;
        params.nodeId = 0;
        params.errorMsg = msg;
        return params;
    };

    int idStartIndex = RobotConstants::COMMANDS::COMMAND_LEN;
    if (command.charAt(idStartIndex) != 'J')
    {
        return fail("INVALID PARAMETERS FOR ZEI. EXPECTED 'J' AT INDEX " + String(idStartIndex));
    }
    if (command.length() <= idStartIndex + 1)
    {
        return fail("INVALID PARAMETERS FOR ZEI. NO NODE IDS PROVIDED.");
    }
    char nodeName = command.charAt(idStartIndex + 1);
    uint8_t nodeId = 0;
    switch (nodeName)
    {
    case 'A':
        nodeId = 1;
        break;
    case 'B':
        nodeId = 2;
        break;
    case 'C':
        nodeId = 3;
        break;
    case 'D':
        nodeId = 4;
        break;
    case 'E':
        nodeId = 5;
        break;
    case 'F':
        nodeId = 6;
        break;
        break;
    default:
        return fail("INVALID PARAMETERS FOR ZEI. EXPECTED NODE ID AFTER 'J' AT INDEX " + String(idStartIndex));
    }

    if (idStartIndex + 2 != command.length())
    {
        return fail("INVALID PARAMETERS FOR ZEI. ONLY SINGLE NODE ID SUPPORTED CURRENTLY.");
    }

    params.nodeId = nodeId;
    params.status = ParamsStatus::OK;
    params.forAllNodes = false;
    params.errorMsg = "";
    return params;
}

int stringToNodeId(String command)
{ // RPP 1
    if (command.equals(RobotConstants::COMMANDS::REQUEST_POSITION))
    {
        return -1;
    }

    String idStr = command.substring(RobotConstants::COMMANDS::COMMAND_LEN);
    bool isValidInteger = true;
    if (idStr.length() == 0)
    {
        return -1;
    }
    else
    {
        for (size_t i = 0; i < idStr.length(); ++i)
        {
            if (!isDigit(idStr.charAt(i)))
            {
                isValidInteger = false;
                break;
            }
        }
    }

    if (!isValidInteger)
    {
        return -1;
    }
    else
    {
        return idStr.toInt();
    }
}

bool handleMove(MoveParams<RobotConstants::Robot::AXIS_COUNT> params, bool isAbsoluteMove)
{
    if (params.status != ParamsStatus::OK)
    {
        if (params.status == ParamsStatus::INVALID_PARAMS)
            addDataToOutQueue("INVALID PARAMS");
        else if (params.status == ParamsStatus::INCORRECT_COMMAND)
            addDataToOutQueue("INCORRECT COMMAND");
        return false;
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
    return true;
}

bool handleSetCurrentPositionInSteps(PositionParams params)
{
    if (params.nodeId < 1 || params.nodeId > RobotConstants::Robot::AXIS_COUNT)
    {
        addDataToOutQueue("INVALID NODE ID: " + String(params.nodeId));
        return false;
    }

    moveController.getAxis(params.nodeId).setCurrentPositionInSteps(params.currentPosition);
    addDataToOutQueue("(S)New current position for " + String(params.nodeId) + ": " + String(moveController.getAxis(params.nodeId).getCurrentPositionInSteps()));
    return true;
}

bool handleSetCurrentPositionInUnits(PositionParams params)
{
    if (params.nodeId < 1 || params.nodeId > moveController.getAxesCount())
    {
        addDataToOutQueue("INVALID NODE ID: " + String(params.nodeId));
        return false;
    }
    moveController.getAxis(params.nodeId).setCurrentPositionInUnits(params.currentPosition);
    addDataToOutQueue("(U)New current position for " + String(params.nodeId) + ": " + String(moveController.getAxis(params.nodeId).getCurrentPositionInSteps()));
    return true;
}

bool handleZeroInitialize(ZEIParams params)
{
    if (params.status != ParamsStatus::OK)
    {
        if (params.errorMsg.length() > 0)
        {
            addDataToOutQueue(params.errorMsg);
        }
        else
        {
            addDataToOutQueue("INVALID PARAMS FOR ZEI");
        }
        return false;
    }

    uint8_t totalNodes = 0;

    if (params.forAllNodes)
    {
        addDataToOutQueue("START ZEI FOR ALL NODES");
        moveController.startZeroInitializationAllAxes();
    }
    else
    {
        addDataToOutQueue("START ZEI FOR NODE " + String(params.nodeId));
        moveController.startZeroInitializationSingleAxis(params.nodeId);
    }
    return true;
}

bool handleRequestPosition(int parsedId)
{
    if (parsedId == -1)
    {
        addDataToOutQueue("RPP COMMAND: INVALID PARAMETERS");
        return false;
    }
    if (parsedId < 1 || RobotConstants::Robot::AXIS_COUNT < parsedId)
    {
        addDataToOutQueue("RPP COMMAND: NODE ID IS INVALID: " + String(parsedId));
        return false;
    }
    uint8_t nodeId = static_cast<uint8_t>(parsedId);

    if (!canOpen.sendSDORead(nodeId, RobotConstants::ODIndices::POSITION_ACTUAL_VALUE, 0))
    {
        addDataToOutQueue("RPP COMMAND: SDO SEND FAILED");
        return false;
    }
    addDataToOutQueue("RPP COMMAND: SDO SEND SUCCESS");
    return true;
}
