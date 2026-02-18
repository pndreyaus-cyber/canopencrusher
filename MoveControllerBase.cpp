#include <algorithm>
#include <cmath>
#include "MoveControllerBase.h"
#include "PrepareMoveMathTestRunner.h"
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
        PrepareMoveComputationResult prepareResult = prepareMove(params);
        if (prepareResult.status == PrepareMoveStatus::NO_EFFECTIVE_MOTION)
        {
            addDataToOutQueue(RobotConstants::Commands::MOVE_ABSOLUTE + " " + RobotConstants::Status::OK + "  | ");
            return true;
        }

        if (prepareResult.status != PrepareMoveStatus::OK)
        {
            return false;
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            MAJ_start(nodeId);
        }

        return true;
    }

    bool MoveControllerBase::isMoveInProgress() const
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            const Axis &axis = axes.at(nodeId);
            if (axis.status == RobotConstants::MoveStatus::PREPARED_FOR_MOVE ||
                axis.status == RobotConstants::MoveStatus::READY_TO_MOVE ||
                axis.status == RobotConstants::MoveStatus::MOVING)
            {
                return true;
            }
        }
        return false;
    }

    bool MoveControllerBase::runPrepareMoveMathTests()
    {
        return runPrepareMoveMathTestSuite(*this, prepareMoveTestVerbose);
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

    MoveControllerBase::PrepareMoveComputationResult MoveControllerBase::computePrepareMove(const MoveParams<RobotConstants::Robot::AXES_COUNT> &params) const
    {
        PrepareMoveComputationResult result;

        int32_t maxMovementAbs = 0;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            double movementUnits = params.movementUnits[nodeId - 1];
            const Axis &axis = axes.at(nodeId);
            PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];

            const int32_t targetSteps = Axis::unitsToSteps(movementUnits);
            const int32_t currentSteps = axis.getCurrentPositionInSteps();
            const int32_t relativeSteps = targetSteps - currentSteps;

            axisResult.requestedMovement = std::abs(movementUnits) > 0.0;
            axisResult.targetSteps = targetSteps;
            axisResult.relativeSteps = relativeSteps;
            axisResult.quantizedToZero = axisResult.requestedMovement && relativeSteps == 0;

            result.hasQuantizedToZero = result.hasQuantizedToZero || axisResult.quantizedToZero;

            int32_t axisRelativeMovementAbsInSteps = std::abs(relativeSteps);
            if (axisRelativeMovementAbsInSteps > maxMovementAbs)
            {
                maxMovementAbs = axisRelativeMovementAbsInSteps;
                result.maxMovementAxisId = nodeId;
            }
        }

        result.maxMovementAbsSteps = maxMovementAbs;
        if (maxMovementAbs == 0)
        {
            result.status = PrepareMoveStatus::NO_EFFECTIVE_MOTION;
            result.reason = result.hasQuantizedToZero ? "NO_EFFECTIVE_MOTION_QUANTIZED" : "NO_EFFECTIVE_MOTION";
            result.syncModelValid = true;
            return result;
        }

        if (params.speed == 0)
        {
            result.status = PrepareMoveStatus::INVALID_SPEED;
            result.reason = "SPEED_IS_ZERO";
            result.syncModelValid = false;
            return result;
        }

        if (params.acceleration == 0)
        {
            result.status = PrepareMoveStatus::INVALID_ACCELERATION;
            result.reason = "ACCELERATION_IS_ZERO";
            result.syncModelValid = false;
            return result;
        }

        const double maxVelocityStepsPerSec = Axis::motorRPMToStepsPerSec(params.speed);
        const double maxAccelerationStepsPerSec2 = Axis::motorRPMPSToStepsPerSec2(params.acceleration);
        if (!std::isfinite(maxVelocityStepsPerSec) || !std::isfinite(maxAccelerationStepsPerSec2) || maxVelocityStepsPerSec <= 0.0 || maxAccelerationStepsPerSec2 <= 0.0)
        {
            result.status = PrepareMoveStatus::INVALID_PROFILE;
            result.reason = "NON_FINITE_PROFILE_INPUT";
            result.syncModelValid = false;
            return result;
        }

        const double dAccelForMaxVelocity = (maxVelocityStepsPerSec * maxVelocityStepsPerSec) / maxAccelerationStepsPerSec2;
        const double maxDistanceSteps = static_cast<double>(maxMovementAbs);
        if (!std::isfinite(dAccelForMaxVelocity) || !std::isfinite(maxDistanceSteps))
        {
            result.status = PrepareMoveStatus::INVALID_PROFILE;
            result.reason = "PROFILE_DISTANCE_INVALID";
            result.syncModelValid = false;
            return result;
        }

        result.isTriangularProfile = maxDistanceSteps <= dAccelForMaxVelocity;
        if (result.isTriangularProfile)
        {
            result.accelerationTimeSec = std::sqrt(maxDistanceSteps / maxAccelerationStepsPerSec2);
            result.constantVelocityTimeSec = 0.0;
            result.fullMovementTimeSec = 2.0 * result.accelerationTimeSec;
        }
        else
        {
            result.accelerationTimeSec = maxVelocityStepsPerSec / maxAccelerationStepsPerSec2;
            result.constantVelocityTimeSec = (maxDistanceSteps - dAccelForMaxVelocity) / maxVelocityStepsPerSec;
            result.fullMovementTimeSec = 2.0 * result.accelerationTimeSec + result.constantVelocityTimeSec;
        }

        if (!std::isfinite(result.accelerationTimeSec) || result.accelerationTimeSec <= 0.0 ||
            !std::isfinite(result.constantVelocityTimeSec) || result.constantVelocityTimeSec < 0.0 ||
            !std::isfinite(result.fullMovementTimeSec) || result.fullMovementTimeSec <= 0.0)
        {
            result.status = PrepareMoveStatus::INVALID_PROFILE;
            result.reason = "PROFILE_TIME_INVALID";
            result.syncModelValid = false;
            return result;
        }

        const double denominator = result.accelerationTimeSec + result.constantVelocityTimeSec;
        if (!std::isfinite(denominator) || denominator <= 0.0)
        {
            result.status = PrepareMoveStatus::INVALID_PROFILE;
            result.reason = "PROFILE_DENOMINATOR_INVALID";
            result.syncModelValid = false;
            return result;
        }

        bool hasEffectiveMotion = false;
        bool syncModelValid = true;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];
            const double relativeAbsSteps = std::abs(static_cast<double>(axisResult.relativeSteps));
            if (relativeAbsSteps == 0.0)
            {
                continue;
            }

            hasEffectiveMotion = true;
            axisResult.velocityStepsPerSec = relativeAbsSteps / denominator;
            axisResult.accelerationStepsPerSec2 = axisResult.velocityStepsPerSec / result.accelerationTimeSec;
            if (!std::isfinite(axisResult.velocityStepsPerSec) || axisResult.velocityStepsPerSec <= 0.0 ||
                !std::isfinite(axisResult.accelerationStepsPerSec2) || axisResult.accelerationStepsPerSec2 <= 0.0)
            {
                syncModelValid = false;
            }

            const double velocityRpmDouble = Axis::stepsPerSecToMotorRPMDouble(axisResult.velocityStepsPerSec);
            const double accelerationRpmPerSecDouble = Axis::stepsPerSec2ToRPMPSDouble(axisResult.accelerationStepsPerSec2);
            if (!std::isfinite(velocityRpmDouble) || !std::isfinite(accelerationRpmPerSecDouble))
            {
                syncModelValid = false;
                continue;
            }

            uint32_t profileVelocityRpm = static_cast<uint32_t>(std::ceil(velocityRpmDouble));
            uint32_t profileAccelerationRpmPerSec = static_cast<uint32_t>(std::ceil(accelerationRpmPerSecDouble));
            if (profileVelocityRpm == 0)
            {
                profileVelocityRpm = 1;
            }
            if (profileAccelerationRpmPerSec == 0)
            {
                profileAccelerationRpmPerSec = 1;
            }

            axisResult.profileVelocityRpm = std::min(profileVelocityRpm, RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_RPM);
            axisResult.profileAccelerationRpmPerSec = std::min(profileAccelerationRpmPerSec, RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S);

            const double axisAccelerationTime = axisResult.velocityStepsPerSec / axisResult.accelerationStepsPerSec2;
            const double axisConstantTime = (relativeAbsSteps / axisResult.velocityStepsPerSec) - axisAccelerationTime;
            const double syncTolerance = 1e-6;
            if (std::abs(axisAccelerationTime - result.accelerationTimeSec) > syncTolerance ||
                std::abs(axisConstantTime - result.constantVelocityTimeSec) > syncTolerance)
            {
                syncModelValid = false;
            }
        }

        if (!hasEffectiveMotion)
        {
            result.status = PrepareMoveStatus::NO_EFFECTIVE_MOTION;
            result.reason = result.hasQuantizedToZero ? "NO_EFFECTIVE_MOTION_QUANTIZED" : "NO_EFFECTIVE_MOTION";
            result.syncModelValid = true;
            return result;
        }

        result.syncModelValid = syncModelValid;
        if (!result.syncModelValid)
        {
            result.status = PrepareMoveStatus::INVALID_PROFILE;
            result.reason = "SYNC_MODEL_INVALID";
            return result;
        }

        result.status = PrepareMoveStatus::OK;
        result.reason = result.hasQuantizedToZero ? "OK_WITH_QUANTIZATION" : "OK";
        return result;
    }

    MoveControllerBase::PrepareMoveComputationResult MoveControllerBase::prepareMove(const MoveParams<RobotConstants::Robot::AXES_COUNT> &params)
    {
        PrepareMoveComputationResult result = computePrepareMove(params);

        if (result.status == PrepareMoveStatus::OK || result.status == PrepareMoveStatus::NO_EFFECTIVE_MOTION)
        {
            for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
            {
                Axis &axis = axes.at(nodeId);
                const PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];

                axis.setTargetPositionInSteps(axisResult.targetSteps);
                axis.setProfileVelocityInRPM(axisResult.profileVelocityRpm);
                axis.setProfileAccelerationInRPMPerSec(axisResult.profileAccelerationRpmPerSec);

                if (result.status == PrepareMoveStatus::NO_EFFECTIVE_MOTION)
                {
                    axis.status = RobotConstants::MoveStatus::OPERATIONAL;
                }
                else
                {
                    axis.status = RobotConstants::MoveStatus::PREPARED_FOR_MOVE;
                }

                DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": target(steps)=" + String(axisResult.targetSteps) + ", rel(steps)=" + String(axisResult.relativeSteps) + ", vel(rpm)=" + String(axisResult.profileVelocityRpm) + ", acc(rpm/s)=" + String(axisResult.profileAccelerationRpmPerSec));
            }

            DBG_INFO(DBG_GROUP_MOVE, "prepareMove status=" + prepareMoveStatusToString(result.status) + ", reason=" + result.reason + ", triangular=" + String(result.isTriangularProfile) + ", ta=" + String(result.accelerationTimeSec) + ", tc=" + String(result.constantVelocityTimeSec) + ", tt=" + String(result.fullMovementTimeSec));
            return result;
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            axes[nodeId].status = RobotConstants::MoveStatus::MOVE_FAILED;
        }
        DBG_ERROR(DBG_GROUP_MOVE, "prepareMove failed: status=" + prepareMoveStatusToString(result.status) + ", reason=" + result.reason);
        return result;
    }

    MoveControllerBase::PrepareMoveComputationResult MoveControllerBase::computePrepareMoveForTesting(const MoveParams<RobotConstants::Robot::AXES_COUNT> &params) const
    {
        return computePrepareMove(params);
    }

    String MoveControllerBase::prepareMoveStatusToString(PrepareMoveStatus status)
    {
        switch (status)
        {
        case PrepareMoveStatus::OK:
            return "OK";
        case PrepareMoveStatus::NO_EFFECTIVE_MOTION:
            return "NO_EFFECTIVE_MOTION";
        case PrepareMoveStatus::INVALID_SPEED:
            return "INVALID_SPEED";
        case PrepareMoveStatus::INVALID_ACCELERATION:
            return "INVALID_ACCELERATION";
        case PrepareMoveStatus::INVALID_PROFILE:
            return "INVALID_PROFILE";
        case PrepareMoveStatus::INVALID_AXIS:
            return "INVALID_AXIS";
        default:
            return "UNKNOWN";
        }
    }

    // ============================ Protected methods end ===========================

    // ============================= Private methods =============================

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
                axis.status = RobotConstants::MoveStatus::FAILED; // Set status to FAILED on heartbeat timeout
            }
            else if ((now - lastHb) <= RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && !axis.isAlive)
            {
                DBG_ERROR(DBG_GROUP_HEARTBEAT, "==== Heartbeat restored for Axis " + String(nodeId) + " ====");
                axis.isAlive = true;
                axis.status = RobotConstants::MoveStatus::OPERATIONAL; // Reset status for the axis when heartbeat is restored
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
            if (axis.status == RobotConstants::MoveStatus::MOVING && !axis.isAlive)
            {
                DBG_WARN(DBG_GROUP_MOVE, "MAJ failed for Axis " + String(nodeId) + ": Heartbeat timeout");
                axis.status = RobotConstants::MoveStatus::MOVE_FAILED;
                MAJ_finalResult();
            }

            uint32_t now = millis();
            if (now - axis.lastRequestedStatusWord > 100 && axis.status == RobotConstants::MoveStatus::MOVING)
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
        canOpen->set_callback_read_x6040_controlword([this](uint8_t callbackNodeId, bool success, uint16_t controlWord)
                                                    { this->MAJ_afterRequestOf_0x6040(callbackNodeId, success, controlWord); }, nodeId);

        // Step 4
        bool successSend = canOpen->sendSDORead(nodeId,
                                                RobotConstants::ODIndices::CONTROLWORD,
                                                RobotConstants::ODIndices::DEFAULT_SUBINDEX);

        // Step 5
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send SDO read request (0x6040) for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_read_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::MAJ_afterRequestOf_0x6040(uint8_t nodeId, bool success, uint16_t controlWord)
    {
        // Step 1
        canOpen->set_callback_read_x6040_controlword(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to read control word (0x6040) for Axis " + String(nodeId)))
        {
            return;
        }

        // Step 3 (data processing)
        if ((controlWord & 0x000F) != 0x000F)
        {
            // Step 4
            canOpen->set_callback_x6040_controlword([this](uint8_t cbNodeId, bool cbSuccess)
                                                    { this->MAJ_afterWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
            
            // Step 5
            bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                                0x000F);
            // Step 6
            if (!MAJ_checkResponseStatus(nodeId, successSend,
                                         "MAJ: Failed to send control word (0x6040) for Axis " + String(nodeId)))
            {
                // Step 7
                canOpen->set_callback_x6040_controlword(nullptr, nodeId);
            }
        } else {
            MAJ_setTargetVelocity(nodeId);
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
        // Step 3 (data processing) -- no processing. Go straight to step 4
        MAJ_setTargetVelocity(nodeId);
    }

    void MoveControllerBase::MAJ_setTargetVelocity(uint8_t nodeId)
    {
        // Step 4
        canOpen->set_callback_x6081_profileVelocity([this](uint8_t cbNodeId, bool cbSuccess)
                                                        { this->MAJ_afterWriteTo_0x6081(cbNodeId, cbSuccess); }, nodeId);
        // Step 5
        bool successSend = canOpen->send_x6081_profileVelocity(nodeId,
                                                            axes[nodeId].getProfileVelocityInRPM());
        // Step 6
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                    "MAJ: Failed to send profile velocity (0x6081) for Axis " + String(nodeId)))
        {
            // Step 7
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
        canOpen->set_callback_TPDO4([this](uint8_t cbNodeId, int32_t actualLocation, uint16_t statusWord)
                                    { this->MAJ_TPDO4(cbNodeId, actualLocation, statusWord); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_RPDO4(nodeId,
                                            axes[nodeId].getTargetPositionInSteps());
        // Step 5
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send RPDO4 for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_TPDO4(nullptr, nodeId);
        }
    }

    void MoveControllerBase::MAJ_TPDO4(uint8_t nodeId, int32_t actualLocation, uint16_t statusWord)
    {
        // Step 1
        canOpen->set_callback_TPDO4(nullptr, nodeId);
        // Step 2
        // if (!MAJ_checkResponseStatus(nodeId, success,
        //                              "MAJ: Failed to set profile acceleration (0x6083) for Axis " + String(nodeId)))
        // {
        //     return;
        // }

        // Step 3 (Data processing)
        DBG_WARN(DBG_GROUP_MOVE, "MAJ TPDO4 from node " + String(nodeId) + ": actualLocation=" + String(actualLocation) + ", statusWord=0x" + String(statusWord, HEX));
        axes[nodeId].status = RobotConstants::MoveStatus::READY_TO_MOVE;
        MAJ_SYNCFunnel();
    }

    void MoveControllerBase::MAJ_SYNCFunnel()
    {

        for(uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            DBG_INFO(DBG_GROUP_MOVE, "MAJ_SYNCFunnel checking Axis " + String(nodeId) + " with status " + String(axes[nodeId].status));
            RobotConstants::MoveStatus status = axes[nodeId].status;
            if(status == RobotConstants::MoveStatus::PREPARED_FOR_MOVE)
            {

                return; // Not all axes are ready yet
            }
        }

        // All axes are ready, send SYNC
        DBG_INFO(DBG_GROUP_MOVE, "MAJ_SYNCFunnel: All axes are ready. Sending SYNC and starting movement.");
        canOpen->sendSYNC();
        delay(10);
        for(uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if(axes[nodeId].status == RobotConstants::MoveStatus::READY_TO_MOVE)
            {
                axes[nodeId].status = RobotConstants::MoveStatus::MOVING;
                DBG_INFO(DBG_GROUP_MOVE, "MAJ_SYNCFunnel: Axis " + String(nodeId) + " status set to MOVING.");
                MAJ_requestStatusWord(nodeId); // Request status immediately after sending SYNC to minimize the delay before we get the first status update
            }
        }
    }

    void MoveControllerBase::MAJ_requestStatusWord(uint8_t nodeId)
    {
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
        //DBG_INFO(DBG_GROUP_MOVE, "MAJ_statusWordCallback called for node " + String(nodeId) + " with success=" + String(success) + " and statusWord=0x" + String(statusWord, HEX));
        // Step 1
        canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to read statusword for Axis " + String(nodeId)))
        {
            return;
        }
        // Step 3 (Data processing)
        //DBG_INFO(DBG_GROUP_MOVE, "MAJ Status Word from node " + String(nodeId) + ": 0x" + String(statusWord, HEX));
        if(MAJ_checkTargetPositionReached(statusWord))
        {
            DBG_INFO(DBG_GROUP_MOVE, "MAJ Target position reached for Axis " + String(nodeId));
            axes[nodeId].status = RobotConstants::MoveStatus::MOVE_FINISHED;
            DBG_INFO(DBG_GROUP_MOVE, "MAJ Movement finished for Axis " + String(nodeId));
            MAJ_finalResult();
            return;
        }
    }

    void MoveControllerBase::MAJ_finalResult() {
        String successfullAxes = "";
        String failedAxes = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].status == RobotConstants::MoveStatus::MOVING || axes[nodeId].status == RobotConstants::MoveStatus::READY_TO_MOVE || axes[nodeId].status == RobotConstants::MoveStatus::PREPARED_FOR_MOVE)
            {
                DBG_INFO(DBG_GROUP_MOVE, "Still going for Axis " + String(nodeId)); 
                return; // Still ongoing for some axes
            }
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.status == RobotConstants::MoveStatus::MOVE_FINISHED)
            {
                axis.status = RobotConstants::MoveStatus::OPERATIONAL; // Reset status for the axis
                successfullAxes += String(nodeId) + " ";
            }
            else if (axis.status == RobotConstants::MoveStatus::MOVE_FAILED)
            {
                axis.status = RobotConstants::MoveStatus::FAILED; // Reset status for the axis
                failedAxes += String(nodeId) + " ";
            }
            else
            {
                DBG_ERROR(DBG_GROUP_MOVE, "MAJ_finalResult called for Axis " + String(nodeId) + " with invalid status: " + String(axis.status));
                axis.status = RobotConstants::MoveStatus::FAILED; // Reset status for the axis
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
            axes[nodeId].status = RobotConstants::MoveStatus::MOVE_FAILED;
            MAJ_finalResult();
        }
        if (!axes[nodeId].isAlive)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "MAJ Failed for Axis " + String(nodeId) + ": Axis is not alive (heartbeat timeout)");
            axes[nodeId].status = RobotConstants::MoveStatus::MOVE_FAILED;
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