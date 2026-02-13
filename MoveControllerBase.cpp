#include <cmath>
#include "MoveControllerBase.h"
#include "Arduino.h"
#include "Debug.h"

namespace StepDirController
{
    // ============================= Public methods =============================

    void MoveControllerBase::requestStatus()
    {
        String reply = RobotConstants::Commands::MOTOR_STATUS + " " + RobotConstants::Status::OK + " ";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes.at(nodeId);
            reply += String(nodeId) + ":" + String(axis.isAlive) + "," + String(axis.initStatus) + +"," + String(axis.status) + "; ";
        }
        addDataToOutQueue(reply);
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

        canOpen->set_callback_heartbeat([this](uint8_t nodeId, uint8_t status)
                                        { this->regularHeartbeatCallback(nodeId, status); });

        initialized = true;
        Serial2.println("MoveControllerBase initialized with " + String(axesCnt) + " axes");
        return true;
    }

    void MoveControllerBase::startZeroInitializationAllAxes()
    {
        DBG_INFO(DBG_GROUP_ZEI, "Start ZEI for all axes");
        zeroInitializeSingleAxis = false;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            startZeroInitializationSingleAxis(nodeId);
        }
    }

    void MoveControllerBase::startZeroInitializationSingleAxis(uint8_t nodeId)
    {
        ZEI_start(nodeId);
    }

    bool MoveControllerBase::move(MoveParams<RobotConstants::Robot::AXES_COUNT> params)
    {
        DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase.cpp move called");

        if (!initialized)
        {
            DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase::move failed. Not initialized");
            return false;
        }
        if (!prepareMove(params))
        {
            return false;
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            MAJ_start(nodeId);
        }

        return true;
    }

    void MoveControllerBase::tick_100()
    {
        if (!initialized)
        {
            return;
        }
        tick_checkMAJStatusWord();
    }

    void MoveControllerBase::tick_500()
    {
        if (!initialized)
        {
            return;
        }
        tick_checkTimeouts();
        tick_checkZEITimeouts();
        tick_requestPosition();
    }

    // ============================= Public methods end =============================

    // ============================ Protected methods =============================

    bool MoveControllerBase::prepareMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params)
    {
        uint8_t maxMovementAbsAxisId = 0;
        double maxMovementAbs = 0;
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            double movementUnits = params.movementUnits[nodeId - 1];
            Axis &axis = axes.at(nodeId);

            axis.setTargetPositionInUnits(movementUnits);
            DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": target position (units): " + String(movementUnits) + ", target position (steps): " + String(axis.getTargetPositionInSteps()) + ", current position (steps): " + String(axis.getCurrentPositionInSteps()));

            int32_t axisRelativeMovementAbsInSteps = std::abs(axis.getRelativeMovementInSteps());
            if (axisRelativeMovementAbsInSteps > maxMovementAbs)
            {
                maxMovementAbs = axisRelativeMovementAbsInSteps;
                maxMovementAbsAxisId = nodeId;
            }
        }

        Axis &maxMovementAbsAxis = axes.at(maxMovementAbsAxisId);
        maxMovementAbsAxis.setProfileVelocityInUnitsPerSec(params.speed);
        DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(maxMovementAbsAxisId) + ": regular speed (units/s): " + String(params.speed) + ", regular speed (RPM): " + String(maxMovementAbsAxis.getProfileVelocityInRPM()));
        maxMovementAbsAxis.setProfileAccelerationInUnitsPerSec2(params.acceleration);
        DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(maxMovementAbsAxisId) + ": regular acceleration (units/s^2): " + String(params.acceleration) + ", regular acceleration (RPM/s): " + String(maxMovementAbsAxis.getProfileAccelerationInRPMPerSec()));
        maxMovementAbsAxis.status = RobotConstants::AxisStatus::PREPARED_FOR_MOVE;

        DBG_INFO(DBG_GROUP_MOVE, "profile velocity (RPM): " + String(maxMovementAbsAxis.getProfileVelocityInRPM()));
        DBG_INFO(DBG_GROUP_MOVE, "profile acceleration (RPM/s): " + String(maxMovementAbsAxis.getProfileAccelerationInRPMPerSec()));
        DBG_INFO(DBG_GROUP_MOVE, "max movement in steps: " + String(Axis::stepsToMotorRevs(maxMovementAbs)));
        double accelerationTimeSec = static_cast<double>(maxMovementAbsAxis.getProfileVelocityInRPM()) / maxMovementAbsAxis.getProfileAccelerationInRPMPerSec();
        double fullMovementTimeSec = Axis::stepsToMotorRevs(maxMovementAbs) * RobotConstants::Math::SECONDS_IN_MINUTE / maxMovementAbsAxis.getProfileVelocityInRPM() + accelerationTimeSec;
        double constantVelocityTimeSec = fullMovementTimeSec - 2 * accelerationTimeSec;

        if (accelerationTimeSec == 0)
        {
            maxMovementAbsAxis.status = RobotConstants::AxisStatus::MOVE_FAILED;
            DBG_ERROR(DBG_GROUP_MOVE, "Acceleration time is zero. This may be a sign of incorrect move parameters (zero speed or zero acceleration).");
            return false;
        }

        if (fullMovementTimeSec == 0)
        {
            maxMovementAbsAxis.status = RobotConstants::AxisStatus::MOVE_FAILED;
            DBG_ERROR(DBG_GROUP_MOVE, "Full movement time is zero. This may be a sign of incorrect move parameters (zero speed or zero acceleration).");
            return false;
        }

        if (constantVelocityTimeSec <= 0)
        {
            maxMovementAbsAxis.status = RobotConstants::AxisStatus::MOVE_FAILED;
            DBG_WARN(DBG_GROUP_MOVE, "Constant velocity time is non-negative. This may be a sign of incorrect move parameters (zero speed or zero acceleration).");
            // return false;
        }

        DBG_INFO(DBG_GROUP_MOVE, "Max movement in steps: " + String(maxMovementAbs) + " for axis " + String(maxMovementAbsAxisId));
        DBG_INFO(DBG_GROUP_MOVE, "Acceleration time (s): " + String(accelerationTimeSec));
        DBG_INFO(DBG_GROUP_MOVE, "Constant velocity time (s): " + String(constantVelocityTimeSec));
        DBG_INFO(DBG_GROUP_MOVE, "Full movement time (s): " + String(fullMovementTimeSec));

        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            Axis &axis = axes.at(nodeId);

            if (nodeId != maxMovementAbsAxisId)
            {
                double velocityInStepsPerSec = Axis::unitsToSteps(std::abs(params.movementUnits[nodeId - 1])) / (constantVelocityTimeSec + accelerationTimeSec);
                axis.setProfileVelocityInRPM(Axis::stepsPerSecToMotorRPM(velocityInStepsPerSec));
                axis.setProfileAccelerationInRPMPerSec(axis.getProfileVelocityInRPM() / accelerationTimeSec);
                DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": velocity (RPM): " + String(axis.getProfileVelocityInRPM()) + ", acceleration (RPM/s): " + String(axis.getProfileAccelerationInRPMPerSec()));
                if (axis.getProfileVelocityInRPM() == 0)
                {
                    DBG_WARN(DBG_GROUP_MOVE, "Axis " + String(nodeId) + " has zero velocity. This may be a sign of incorrect move parameters.");
                }
                axis.status = RobotConstants::AxisStatus::PREPARED_FOR_MOVE;
            }
        }
        DBG_WARN(DBG_GROUP_MOVE, "Move prepared. Note: if the move failed due to incorrect parameters, some axes may have status MOVE_FAILED. Check logs for details.");
        return true;
    }

    // void MoveControllerBase::prepareMove() // TODO: Does not work for a = 0, maybe other corner cases
    //     {
    //         if (axesCnt == 0 || axes.empty())
    //         {
    //             addDataToOutQueue("No axes configured");
    //             return;
    //         }
    //         DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase.cpp prepareMove called");
    //         uint8_t maxMovementAxisId = 0;
    //         bool firstAxis = true;
    //         double maxMovement = 0;

    //         for (auto it = axes.begin(); it != axes.end(); ++it)
    //         {
    //             Axis &axis = it->second;
    //             double axisMovement = std::fabs(axis.getMovementUnits());
    //             if (firstAxis || axisMovement > maxMovement)
    //             {
    //                 maxMovement = axisMovement;
    //                 maxMovementAxisId = axis.nodeId;
    //                 firstAxis = false;
    //             }
    //         }

    //         if (accelerationUnits == 0)
    //         { // Right now we do not support zero acceleration. But in the future we can add special handling for this case.
    //             addDataToOutQueue("MoveControllerBase.cpp zero acceleration is not supported");
    //             return;
    //         }
    //         double tAcceleration = regularSpeedUnits / accelerationUnits; // в секундах
    //         if (regularSpeedUnits == 0)
    //         { // Zero speed means no movement at all. It is strange to call move with zero speed
    //             addDataToOutQueue("MoveControllerBase.cpp zero regularSpeedUnits is not supported (zero speed)");
    //             return;
    //         }
    //         double tCruising = (maxMovement - regularSpeedUnits * regularSpeedUnits / accelerationUnits) / regularSpeedUnits;
    //         double res = 0;

    //         if (maxMovement == 0)
    //         {
    //             addDataToOutQueue("MoveControllerBase.cpp maxMovement division by zero. Motors do not need to move.");
    //             return;
    //         }

    //         if (tAcceleration == 0 || (tAcceleration + tCruising == 0))
    //         {
    //             addDataToOutQueue("MoveControllerBase.cpp tAcceleration or tAcceleration + tCruising division by zero");
    //             return;
    //         }

    //         for (auto it = axes.begin(); it != axes.end(); ++it)
    //         {
    //             Axis &axis = it->second;

    //             double axisMovementUnits = std::fabs(axis.getMovementUnits());

    //             axis.acceleration = (axisMovementUnits) / (tAcceleration * (tAcceleration + tCruising));
    //             axis.regularSpeed = axis.acceleration * tAcceleration;
    //             axis.params.x6083_profileAcceleration = axis.accelerationUnitsTorpmPerSecond(axis.acceleration);
    //             axis.params.x6081_profileVelocity = axis.speedUnitsToRevolutionsPerMinute(axis.regularSpeed);
    //         }
    //     }
    // ============================ Protected methods end ===========================

    // ============================= Private methods =============================
    // void MoveControllerBase::sendMove()
    // {
    //     for (auto it = axes.begin(); it != axes.end(); ++it)
    //     {
    //         Axis &axis = it->second;
    //         canOpen->send_x6081_profileVelocity(axis.nodeId, axis.params.x6081_profileVelocity);
    //         canOpen->send_x6083_profileAcceleration(axis.nodeId, axis.params.x6083_profileAcceleration);

    //         canOpen->send_x6040_controlword(axis.nodeId,
    //                                         0x004F);

    //         canOpen->send_x6040_controlword(axis.nodeId,
    //                                         0x005F);

    //         canOpen->sendPDO4_x607A_SyncMovement(axis.nodeId, axis.getTargetPositionAbsolute());

    //         // Imitation, that the motor reached the target position
    //         axis.setCurrentPositionInSteps(axis.params.x607A_targetPosition); // WRONG.
    //         // axis.params.x6064_positionActualValue = axis.params.x607A_targetPosition;
    //     }

    //     delay(5);
    //     // canOpen->sendSYNC();
    // }

    void MoveControllerBase::positionUpdate(uint8_t nodeId, int32_t position)
    {
        DBG_INFO(DBG_GROUP_CANOPEN, "Position update from node " + String(nodeId) + ": " + String(position));
        auto it = axes.find(nodeId);
        if (it != axes.end())
        {
            Axis &axis = it->second;

            axis.setCurrentPositionInSteps(position);
        }
    }

    void MoveControllerBase::setRegularPositionActualValueCallback(uint8_t nodeId)
    {
        canOpen->set_callback_x6064_positionActualValue([this](uint8_t callbackNodeId, bool success, int32_t position)
                                                        { this->regularPositionActualValueCallback(callbackNodeId, success, position); }, nodeId);
    }

    // ======== Timer functions ========
    void MoveControllerBase::tick_checkTimeouts()
    {
        const uint32_t now = millis();
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            const uint32_t lastHb = axis.lastHeartbeatMs;
            if (lastHb == 0)
            {
                continue; // No heartbeat received yet for this axis
            }

            if ((now - lastHb) > RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && axis.isAlive)
            {
                DBG_ERROR(DBG_GROUP_HEARTBEAT, "==== Heartbeat timeout for Axis " + String(nodeId) + " ====");
                axis.isAlive = false;
                axis.status = RobotConstants::AxisStatus::FAILED; // Set status to FAILED on heartbeat timeout
            }
            else if ((now - lastHb) <= RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && !axis.isAlive)
            {
                DBG_ERROR(DBG_GROUP_HEARTBEAT, "==== Heartbeat restored for Axis " + String(nodeId) + " ====");
                axis.isAlive = true;
                axis.status = RobotConstants::AxisStatus::OPERATIONAL; // Reset status for the axis when heartbeat is restored
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
                ZEI_finalResult();
            }
        }
    }

    void MoveControllerBase::tick_requestPosition()
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].isAlive)
            {
                canOpen->sendSDORead(nodeId,
                                     RobotConstants::ODIndices::POSITION_ACTUAL_VALUE,
                                     RobotConstants::ODIndices::DEFAULT_SUBINDEX);
            }
        }
    }

    void MoveControllerBase::tick_checkMAJStatusWord()
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.status == RobotConstants::AxisStatus::MOVING && !axis.isAlive)
            {
                DBG_WARN(DBG_GROUP_MOVE, "MAJ failed for Axis " + String(nodeId) + ": Heartbeat timeout");
                axis.status = RobotConstants::AxisStatus::MOVE_FAILED;
                MAJ_finalResult();
            }

            uint32_t now = millis();
            if (now - axis.lastRequestedStatusWord > 100 && axis.status == RobotConstants::AxisStatus::MOVING)
            {
                // Step 4
                canOpen->set_callback_read_x6041_statusword([this](uint8_t cbNodeId, bool success, uint16_t statusWord)
                                            { this->MAJ_statusWordCallback(cbNodeId, success, statusWord); }, nodeId);
                // Step 5
                bool successSend = canOpen->sendSDORead(nodeId,
                                                        RobotConstants::ODIndices::STATUSWORD,
                                                        RobotConstants::ODIndices::DEFAULT_SUBINDEX);
                // Step 6
                if (!MAJ_checkResponseStatus(nodeId, successSend,
                                            "MAJ: Failed to send statusword request for Axis " + String(nodeId)))
                {
                    // Step 7
                    canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
                }
                
                axes[nodeId].lastRequestedStatusWord = now;
            }
        }
    }

    // ======== Timer functions end ========

    // ======== ZEI Sequence ========
    void MoveControllerBase::ZEI_start(uint8_t nodeId)
    {
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_ONGOING;
        if (zeroInitializeSingleAxis)
        {
            axisToInitialize = nodeId;
        }

        // Step 3
        canOpen->set_callback_x6040_controlword([this](uint8_t callbackNodeId, bool success)
                                                { this->ZEI_AfterFirstWriteTo_0x6040(callbackNodeId, success); }, nodeId);

        // Step 4
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x0000);

        // Step 5
        if (!ZEI_checkResponseStatus(nodeId, successSend,
                                     "ZEI: Failed to send control word <- 0x0000"))
        {
            // Step 6
            canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZEI_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        // Step 2
        if (!ZEI_checkResponseStatus(nodeId, success,
                                     "ZEI: Failed to write 0x0000 to 0x6040"))
        {
            return;
        }
        // Step 3
        canOpen->set_callback_x260A_electronicGearMolecules([this](uint8_t cbNodeId, bool cbSuccess)
                                                            { this->ZEI_AfterFirstWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA66);
        // Step 5
        if (!ZEI_checkResponseStatus(nodeId, successSend,
                                     "ZEI: Failed to send electronic gear molecules <- 0xEA66"))
        {
            // Step 6
            canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZEI_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        // Step 2
        if (!ZEI_checkResponseStatus(nodeId, success,
                                     "ZEI: Failed to write 0xEA66 to 0x260A"))
        {
            return;
        }
        // Step 3
        canOpen->set_callback_x260A_electronicGearMolecules([this](uint8_t cbNodeId, bool cbSuccess)
                                                            { this->ZEI_AfterSecondWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA70);
        // Step 5
        if (!ZEI_checkResponseStatus(nodeId, successSend,
                                     "ZEI: Failed to send electronic gear molecules <- 0xEA70"))
        {
            // Step 6
            canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZEI_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        // Step 2
        if (!ZEI_checkResponseStatus(nodeId, success,
                                     "ZEI: Failed to write 0xEA70 to 0x260A"))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x6040_controlword([this](uint8_t cbNodeId, bool cbSuccess)
                                                { this->ZEI_AfterSecondWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        delay(200);
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x000F);
        // Step 5
        if (!ZEI_checkResponseStatus(nodeId, successSend,
                                     "ZEI: Failed to send control word <- 0x000F"))
        {
            // Step 6
            canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZEI_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        // Step 2
        if (!ZEI_checkResponseStatus(nodeId, success,
                                     "ZEI: Failed to write 0x000F to 0x6040"))
        {
            return;
        }
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FINISHED;
        ZEI_finalResult();
    }

    void MoveControllerBase::ZEI_finalResult()
    {
        if (zeroInitializeSingleAxis)
        {
            String status;
            Axis &axis = axes[axisToInitialize];
            if (axis.initStatus == RobotConstants::InitStatus::ZEI_FINISHED)
            {
                status = RobotConstants::Status::OK;
            }
            else if (axis.initStatus == RobotConstants::InitStatus::ZEI_FAILED)
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

    bool MoveControllerBase::ZEI_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_ZEI, "ZEI Failed for Axis " + String(nodeId) + ": " + errorMessage);
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FAILED;
            ZEI_finalResult();
        }
        return success;
    }
    // ======== ZEI Sequence End ========

    // ======== MAJ Sequence ========
    void MoveControllerBase::MAJ_start(uint8_t nodeId)
    {
        // Step 3
        canOpen->set_callback_x6081_profileVelocity([this](uint8_t callbackNodeId, bool success)
                                                    { this->MAJ_afterWriteTo_0x6081(callbackNodeId, success); }, nodeId);

        // Step 4
        bool successSend = canOpen->send_x6081_profileVelocity(nodeId,
                                                               axes[nodeId].getProfileVelocityInRPM());

        // Step 5
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send profile velocity (0x6081) for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x6081_profileVelocity(nullptr, nodeId);
        }
    }

    void MoveControllerBase::MAJ_afterWriteTo_0x6081(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6081_profileVelocity(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to set profile velocity (0x6081) for Axis " + String(nodeId)))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x6040_controlword([this](uint8_t cbNodeId, bool cbSuccess)
                                                        { this->MAJ_afterWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0xF);
        // Step 5
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send control word (0x6040) for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::MAJ_afterWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to set control word (0x6040) for Axis " + String(nodeId)))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x6083_profileAcceleration([this](uint8_t cbNodeId, bool cbSuccess)
                                                        { this->MAJ_afterWriteTo_0x6083(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x6083_profileAcceleration(nodeId,
                                                                   axes[nodeId].getProfileAccelerationInRPMPerSec());
        // Step 5
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send profile acceleration (0x6083) for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x6083_profileAcceleration(nullptr, nodeId);
        }
    }


    void MoveControllerBase::MAJ_afterWriteTo_0x6083(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6083_profileAcceleration(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to set profile acceleration (0x6083) for Axis " + String(nodeId)))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_TPDO1([this](uint8_t cbNodeId, int32_t actualLocation, uint16_t statusWord)
                                    { this->MAJ_TPDO1(cbNodeId, actualLocation, statusWord); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_RPDO1(nodeId,
                                            0x1F,
                                            0x1,
                                            axes[nodeId].getTargetPositionInSteps());
        // Step 5
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send RPDO1 for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_TPDO1(nullptr, nodeId);
        }
    }

    void MoveControllerBase::MAJ_TPDO1(uint8_t nodeId, int32_t actualLocation, uint16_t statusWord)
    {
        // Step 1
        canOpen->set_callback_TPDO1(nullptr, nodeId);
        // Step 2
        // if (!MAJ_checkResponseStatus(nodeId, success,
        //                              "MAJ: Failed to set profile acceleration (0x6083) for Axis " + String(nodeId)))
        // {
        //     return;
        // }

        // Step 3 (Data processing)
        DBG_WARN(DBG_GROUP_MOVE, "MAJ TPDO1 from node " + String(nodeId) + ": actualLocation=" + String(actualLocation) + ", statusWord=0x" + String(statusWord, HEX));
        
        // Check 10-th bit of the status word (0x400) to determine if the movement is finished
        if (MAJ_checkTargetPositionReached(statusWord))
        {
            axes[nodeId].status = RobotConstants::AxisStatus::MOVE_FINISHED;
            DBG_INFO(DBG_GROUP_MOVE, "MAJ Movement finished for Axis " + String(nodeId));
            MAJ_finalResult();
            return;
        }

        axes[nodeId].status = RobotConstants::AxisStatus::MOVING;
        // Step 4
        canOpen->set_callback_read_x6041_statusword([this](uint8_t cbNodeId, bool success, uint16_t statusWord)
                                    { this->MAJ_statusWordCallback(cbNodeId, success, statusWord); }, nodeId);
        // Step 5
        bool successSend = canOpen->sendSDORead(nodeId,
                                                RobotConstants::ODIndices::STATUSWORD,
                                                RobotConstants::ODIndices::DEFAULT_SUBINDEX);
        // Step 6
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send statusword request for Axis " + String(nodeId)))
        {
            // Step 7
            canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
        }
        
        axes[nodeId].lastRequestedStatusWord = millis();
    }

    void MoveControllerBase::MAJ_statusWordCallback(uint8_t nodeId, bool success, uint16_t statusWord) {
        // Step 1
        canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to read statusword for Axis " + String(nodeId)))
        {
            return;
        }
        // Step 3 (Data processing)
        if(MAJ_checkTargetPositionReached(statusWord))
        {
            axes[nodeId].status = RobotConstants::AxisStatus::MOVE_FINISHED;
            DBG_INFO(DBG_GROUP_MOVE, "MAJ Movement finished for Axis " + String(nodeId));
            canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
            MAJ_finalResult();
            return;
        }
    }

    void MoveControllerBase::MAJ_finalResult() {
        String successfullAxes = "";
        String failedAxes = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].status == RobotConstants::AxisStatus::MOVING)
            {
                return; // Still ongoing for some axes
            }
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.status == RobotConstants::AxisStatus::MOVE_FINISHED)
            {
                axis.status = RobotConstants::AxisStatus::OPERATIONAL; // Reset status for the axis
                successfullAxes += String(nodeId) + " ";
            }
            else if (axis.status == RobotConstants::AxisStatus::MOVE_FAILED)
            {
                axis.status = RobotConstants::AxisStatus::FAILED; // Reset status for the axis
                failedAxes += String(nodeId) + " ";
            }
            else
            {
                DBG_ERROR(DBG_GROUP_MOVE, "MAJ_finalResult called for Axis " + String(nodeId) + " with invalid status");
                axis.status = RobotConstants::AxisStatus::FAILED; // Reset status for the axis
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

        String commandReply = RobotConstants::Commands::MOVE_ABSOLUTE + " " + status + " " + successfullAxes + "|" + failedAxes;
        addDataToOutQueue(commandReply);
    }

    bool MoveControllerBase::MAJ_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "MAJ Failed for Axis " + String(nodeId) + ": " + errorMessage);
            axes[nodeId].status = RobotConstants::AxisStatus::MOVE_FAILED;
            MAJ_finalResult();
        }
        return success;
    }

    bool MoveControllerBase::MAJ_checkTargetPositionReached(uint16_t statusWord)
    {   
        // Check 10-th bit of the status word (0x400) to determine if the movement is finished
        return (statusWord & 0b10000000000) != 0;
    }
    // ======== MAJ Sequence End ========

    // ======== Regular callbacks ========
    void MoveControllerBase::regularHeartbeatCallback(uint8_t nodeId, uint8_t status)
    {
        /*
        HBStatus statusStr;
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
        */
        axes[nodeId].lastHeartbeatMs = millis();
    }

    void MoveControllerBase::regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_CANOPEN, "Failed to read Position Actual Value for node " + String(nodeId));
            return;
        }
        positionUpdate(nodeId, position);
        axes[nodeId].lastHeartbeatMs = millis();
    }
    // ======== Regular callbacks end ========
    // ============================= Private methods end =============================

}