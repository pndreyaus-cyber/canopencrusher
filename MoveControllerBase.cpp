#include <cmath>
#include "MoveControllerBase.h"
#include "Arduino.h"
#include "Debug.h"

namespace StepDirController
{
    void MoveControllerBase::setRegularSpeedUnits(double speed)
    {
        regularSpeedUnits = std::fabs(speed); // Edited for C++
    }

    void MoveControllerBase::setAccelerationUnits(double acceleration)
    {
        accelerationUnits = std::fabs(acceleration); // Edited for C++
    }

    void MoveControllerBase::prepareMove() // TODO: Does not work for a = 0, maybe other corner cases
    {
        if (axesCnt == 0 || axes.empty())
        {
            addDataToOutQueue("No axes configured");
            return;
        }
        DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase.cpp prepareMove called");
        uint8_t maxMovementAxisId = 0;
        bool firstAxis = true;
        double maxMovement = 0;

        for (auto it = axes.begin(); it != axes.end(); ++it)
        {
            Axis &axis = it->second;
            double axisMovement = std::fabs(axis.getMovementUnits());
            if (firstAxis || axisMovement > maxMovement)
            {
                maxMovement = axisMovement;
                maxMovementAxisId = axis.nodeId;
                firstAxis = false;
            }
        }

        if (accelerationUnits == 0)
        { // Right now we do not support zero acceleration. But in the future we can add special handling for this case.
            addDataToOutQueue("MoveControllerBase.cpp zero acceleration is not supported");
            return;
        }
        double tAcceleration = regularSpeedUnits / accelerationUnits; // в секундах
        if (regularSpeedUnits == 0)
        { // Zero speed means no movement at all. It is strange to call move with zero speed
            addDataToOutQueue("MoveControllerBase.cpp zero regularSpeedUnits is not supported (zero speed)");
            return;
        }
        double tCruising = (maxMovement - regularSpeedUnits * regularSpeedUnits / accelerationUnits) / regularSpeedUnits;
        double res = 0;

        if (maxMovement == 0)
        {
            addDataToOutQueue("MoveControllerBase.cpp maxMovement division by zero. Motors do not need to move.");
            return;
        }

        if (tAcceleration == 0 || (tAcceleration + tCruising == 0))
        {
            addDataToOutQueue("MoveControllerBase.cpp tAcceleration or tAcceleration + tCruising division by zero");
            return;
        }

        for (auto it = axes.begin(); it != axes.end(); ++it)
        {
            Axis &axis = it->second;

            double axisMovementUnits = std::fabs(axis.getMovementUnits());

            axis.acceleration = (axisMovementUnits) / (tAcceleration * (tAcceleration + tCruising));
            axis.regularSpeed = axis.acceleration * tAcceleration;
            axis.params.x6083_profileAcceleration = axis.accelerationUnitsTorpmPerSecond(axis.acceleration);
            axis.params.x6081_profileVelocity = axis.speedUnitsToRevolutionsPerMinute(axis.regularSpeed);
        }
    }

    void MoveControllerBase::sendMove()
    {
        for (auto it = axes.begin(); it != axes.end(); ++it)
        {
            Axis &axis = it->second;
            canOpen->send_x6081_profileVelocity(axis.nodeId, axis.params.x6081_profileVelocity);
            canOpen->send_x6083_profileAcceleration(axis.nodeId, axis.params.x6083_profileAcceleration);

            canOpen->send_x6040_controlword(axis.nodeId,
                                            0x004F);

            canOpen->send_x6040_controlword(axis.nodeId,
                                            0x005F);

            canOpen->sendPDO4_x607A_SyncMovement(axis.nodeId, axis.getTargetPositionAbsolute());

            // Imitation, that the motor reached the target position
            axis.setCurrentPositionInSteps(axis.params.x607A_targetPosition);
            // axis.params.x6064_positionActualValue = axis.params.x607A_targetPosition;
        }

        delay(5);
        // canOpen->sendSYNC();
    }

    double MoveControllerBase::getRegularSpeedUnits() const
    {
        return regularSpeedUnits;
    }

    double MoveControllerBase::getAccelerationUnits() const
    {
        return accelerationUnits;
    }

    bool MoveControllerBase::start(CanOpen *canOpen, uint8_t axesCnt)
    {
        if (axesCnt == 0)
        {
            addDataToOutQueue("MoveControllerBase start with 0 axes. This is not allowed");
            return false;
        }
        if (canOpen == nullptr)
        {
            addDataToOutQueue("MoveControllerBase start with nullptr canOpen. This is not allowed");
            return false;
        }

        this->canOpen = canOpen;
        this->axesCnt = axesCnt;

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            axes[nodeId] = Axis(nodeId);
            axes[nodeId].lastHeartbeatMs = 0;
            axes[nodeId].isAlive = false;

            setRegularPositionActualValueCallback(nodeId);
        }

        canOpen->setHeartbeatCallback([this](uint8_t nodeId, uint8_t status)
                                      { this->regularHeartbeatCallback(nodeId, status); });

        initialized = true;
        Serial2.println("MoveControllerBase initialized with " + String(axesCnt) + " axes");
        return true;
    }

    void MoveControllerBase::move()
    {
        DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase.cpp move called");

        if (!initialized)
        {
            DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase::move failed. Not initialized");
            return;
        }
        prepareMove();
        sendMove();
    }

    void MoveControllerBase::tick_requestPosition(const std::vector<uint8_t> &nodeIds)
    {
        if (nodeIds.empty())
        {
            for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
            {
                canOpen->sendSDORead(nodeId,
                                     RobotConstants::ODIndices::POSITION_ACTUAL_VALUE,
                                     RobotConstants::ODIndices::DEFAULT_SUBINDEX);
            }
            return;
        }

        for (uint8_t nodeId : nodeIds)
        {
            canOpen->sendSDORead(nodeId,
                                 RobotConstants::ODIndices::POSITION_ACTUAL_VALUE,
                                 RobotConstants::ODIndices::DEFAULT_SUBINDEX);
        }
    }

    void MoveControllerBase::positionUpdate(uint8_t nodeId, int32_t position)
    {
        DBG_WARN(DBG_GROUP_CANOPEN, "Position update from node " + String(nodeId) + ": " + String(position));
        auto it = axes.find(nodeId);
        if (it != axes.end())
        {
            Axis &axis = it->second;

            axis.setCurrentPositionInSteps(position);
        }
    }

    void MoveControllerBase::setRegularPositionActualValueCallback(uint8_t nodeId)
    {
        canOpen->setPositionActualValueCallback_0x6064([this](uint8_t callbackNodeId, bool success, int32_t position)
                                                       { this->regularPositionActualValueCallback(callbackNodeId, success, position); }, nodeId);
    }

    void MoveControllerBase::startZeroInitializationAllAxes()
    {
        DBG_INFO(DBG_GROUP_ZEI, "Start ZEI for all axes");
        zeroInitializeSingleAxis = false;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            startZeroInitializationSingleAxis(nodeId);
            // delay(1000);
        }
    }

    void MoveControllerBase::startZeroInitializationSingleAxis(uint8_t nodeId)
    {
        zeroInitialize_start(nodeId);
    }

    bool MoveControllerBase::checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_ZEI, "ZEI Failed for Axis " + String(nodeId) + ": " + errorMessage);
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FAILED;
            zeroInitialize_finalResult();
        }
        return success;
    }
    // Callbacks
    /*
    Acknowldegement callbacks should have this structure:
    1) Set the current callback to either nullptr or to the regular callback (if applicable)
    2) Check the response status using checkResponseStatus(). If it fails, return from the function
    3) Set the next callback
    4) Send the next command
    5) Pass status of sending to checkResponseStatus()
    6) If sending fails, set the next callback to nullptr or regular callback (if applicable)

    Read response callbacks should have this structure:
    1) Set the current callback to either nullptr or to the regular callback (if applicable
    2) Check the response status using checkResponseStatus(). If it fails, return from the function
    3) Process received data
    4) Set the next callback
    5) Send the next command
    6) Pass status of sending to checkResponseStatus()
    7) If sending fails, set the next callback to nullptr or regular callback (if applicable)

    So read response callbacks differ from acknowldegement callbacks by steps 3 (process data) only.


    In the end of the sequence, you can insert a funneling function, which will track if every axis has finished and report the final result
*/

    // ============= ZEI START =============
    void MoveControllerBase::zeroInitialize_start(uint8_t nodeId)
    {
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_ONGOING;
        if (zeroInitializeSingleAxis)
        {
            axisToInitialize = nodeId;
        }

        // Step 3
        canOpen->setControlWordWriteStatusCallback_0x6040([this](uint8_t callbackNodeId, bool success)
                                                          { this->zeroInitialize_AfterFirstWriteTo_0x6040(callbackNodeId, success); }, nodeId);

        // Step 4
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x0000);

        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "ZEI: Failed to send control word <- 0x0000"))
        {
            // Step 6
            canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "ZEI: Failed to write 0x0000 to 0x6040"))
        {
            return;
        }
        // Step 3
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A([this](uint8_t cbNodeId, bool cbSuccess)
                                                                      { this->zeroInitialize_AfterFirstWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA66);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "ZEI: Failed to send electronic gear molecules <- 0xEA66"))
        {
            // Step 6
            canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "ZEI: Failed to write 0xEA66 to 0x260A"))
        {
            return;
        }
        // Step 3
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A([this](uint8_t cbNodeId, bool cbSuccess)
                                                                      { this->zeroInitialize_AfterSecondWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA70);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "ZEI: Failed to send electronic gear molecules <- 0xEA70"))
        {
            // Step 6
            canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "ZEI: Failed to write 0xEA70 to 0x260A"))
        {
            return;
        }

        // Step 3
        canOpen->setControlWordWriteStatusCallback_0x6040([this](uint8_t cbNodeId, bool cbSuccess)
                                                          { this->zeroInitialize_AfterSecondWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        delay(200);
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x000F);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "ZEI: Failed to send control word <- 0x000F"))
        {
            // Step 6
            canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "ZEI: Failed to write 0x000F to 0x6040"))
        {
            return;
        }
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FINISHED;
        zeroInitialize_finalResult();
    }

    void MoveControllerBase::zeroInitialize_finalResult()
    {
        if (zeroInitializeSingleAxis)
        {
            String status;
            if (axes[axisToInitialize].initStatus == RobotConstants::InitStatus::ZEI_FINISHED)
            {
                status = RobotConstants::Status::OK;
            }
            else if (axes[axisToInitialize].initStatus == RobotConstants::InitStatus::ZEI_FAILED)
            {
                status = RobotConstants::Status::COMMAND_FULL_FAIL;
            }
            else
            {
                status = RobotConstants::Status::UNKNOWN_ERROR;
            }

            String commandReply = RobotConstants::Commands::ZERO_INITIALIZE + " " + status + " " + String(axisToInitialize);
            addDataToOutQueue(commandReply);

            axisToInitialize = 0;
            return;
        }

        String successfullAxes = "";
        String failedAxes = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_ONGOING)
            {
                return; // Still ongoing for some axes
            }
            else if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_FINISHED)
            {
                successfullAxes += String(nodeId) + " ";
            }
            else if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_FAILED)
            {
                failedAxes += String(nodeId) + " ";
            }
        }

        String status;
        if (failedAxes.length() > 0 && successfullAxes.length() > 0)
        {
            status = RobotConstants::Status::COMMAND_PARTIAL_FAIL;
        }
        else if (failedAxes.length() > 0)
        {
            status = RobotConstants::Status::COMMAND_FULL_FAIL;
        }
        else if (successfullAxes.length() > 0)
        {
            status = RobotConstants::Status::OK;
        }

        zeroInitializeSingleAxis = true; // Reset to default for the next ZEI command

        String commandReply = RobotConstants::Commands::ZERO_INITIALIZE + " " + status + " " + successfullAxes + "|" + failedAxes;
        addDataToOutQueue(commandReply);
    }

    // ============= ZEI END =============

    void MoveControllerBase::regularHeartbeatCallback(uint8_t nodeId, uint8_t status)
    {
        String statusStr;
        if (status == 0x05)
        {
            statusStr = "operational";
        }
        else if (status == 0x04)
        {
            statusStr = "alarm";
        }
        else if (status == 0x7F)
        {
            statusStr = "pre-operational";
        }
        else if (status == 0x00)
        {
            statusStr = "boot-up";
        }
        else
        {
            statusStr = "unknown";
        }
        DBG_INFO(DBG_GROUP_HEARTBEAT, "HB from " + String(nodeId) + ": " + statusStr);
        axes[nodeId].lastHeartbeatMs = millis();
    }

    void MoveControllerBase::tick()
    {
        if (!initialized)
        {
            return;
        }
        tick_checkTimeouts();
        tick_checkZEITimeouts();
        //tick_requestPosition();
        
    }

    void MoveControllerBase::tick_checkTimeouts()
    {
        const uint32_t now = millis();
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            const uint32_t lastHb = axis.lastHeartbeatMs;

            if ((now - lastHb) > RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && axis.isAlive)
            {
                DBG_WARN(DBG_GROUP_HEARTBEAT, "==== Heartbeat timeout for Axis " + String(nodeId) + " ====");
                axis.isAlive = false;
            }
            else if ((now - lastHb) <= RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && !axis.isAlive)
            {
                DBG_WARN(DBG_GROUP_HEARTBEAT, "==== Heartbeat restored for Axis " + String(nodeId) + " ====");
                axis.isAlive = true;
            }
        }
    }

    void MoveControllerBase::tick_checkZEITimeouts()
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.initStatus == RobotConstants::InitStatus::ZEI_ONGOING && !axis.isAlive)
            {
                DBG_WARN(DBG_GROUP_ZEI, "Zero Initialization failed for Axis " + String(nodeId) + ": Heartbeat timeout");
                axis.initStatus = RobotConstants::InitStatus::ZEI_FAILED;
                zeroInitialize_finalResult();
            }
        }
    }

    void MoveControllerBase::regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_CANOPEN, "Failed to read Position Actual Value for node " + String(nodeId));
            return;
        }
        positionUpdate(nodeId, position);
    }

}