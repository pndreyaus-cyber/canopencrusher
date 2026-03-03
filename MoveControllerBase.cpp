#include <algorithm>
#include <cmath>
#include <utility>

#include "MoveControllerBase.h"
#include "PrepareMoveTest.h"
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
            reply += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + " " + String(axis.status) + "," + String(axis.initStatus) + "," + String(axis.moveStatus) + "; ";
        }
        addDataToOutQueue(reply);
    }

    std::optional<int32_t> MoveControllerBase::axisPosition(uint8_t nodeId)
    {
        if (nodeId < 1 || RobotConstants::Robot::AXES_COUNT < nodeId)
        {
            return std::nullopt;
        }
        return axes.at(nodeId).getCurrentPositionInSteps().value();
    }

    ParamsStatusStruct MoveControllerBase::start(CanOpen *canOpen, uint8_t axesCnt, bool writeNewLimitsToEEPROM, uint8_t *nodesToInvert, uint8_t nodesToInvertCnt)
    {
        ParamsStatusStruct status;
        if (axesCnt == 0)
        {
            status.errorMsg = "MoveControllerBase start with 0 axes. This is not allowed";
            status.status = ParamsStatus::INVALID_PARAMS;
            return status;
        }
        if (canOpen == nullptr)
        {
            status.errorMsg = "MoveControllerBase start with nullptr canOpen. This is not allowed";
            status.status = ParamsStatus::INVALID_PARAMS;
            return status;
        }

        this->canOpen = canOpen;
        this->axesCnt = axesCnt;

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            axes[nodeId] = Axis(nodeId);
            axes[nodeId].lastHeartbeatMs = 0;
            axes[nodeId].limitsEnabled = true;
            axes[nodeId].setLimits(
                RobotConstants::Axis::DEFAULT_MIN_LIMITS[nodeId - 1],
                RobotConstants::Axis::DEFAULT_MAX_LIMITS[nodeId - 1]);
            setRegularPositionActualValueCallback(nodeId);
        }
        for (uint8_t i = 0; i < nodesToInvertCnt; ++i)
        {
            uint8_t nodeId = nodesToInvert[i];
            if (nodeId >= 1 && nodeId <= axesCnt)
            {
                axes[nodeId].reverseLogic();
                DBG_INFO(DBG_GROUP_INIT, "Axis " + String(nodeId) + " is set to be inverted");
            }
        }

        canOpen->set_callback_heartbeat([this](uint8_t nodeId, uint8_t status)
                                        { this->regularHeartbeatCallback(nodeId, status); });

        initialized = true;

        return status;
    }

    void MoveControllerBase::startZeroInitializationAllAxes()
    {
        DBG_VERBOSE(DBG_GROUP_ZEI, "Start ZEI for all axes");
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

    MoveControllerBase::PrepareMoveStatus MoveControllerBase::move(MoveParams<RobotConstants::Robot::AXES_COUNT> params, bool isAbsoluteMove, const String *commandNameForLogging)
    {
        if (!initialized)
        {
            DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase::move failed. Not initialized");
            return PrepareMoveStatus::NOT_INITIALIZED;
        }
        if (isMAJInProgress)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "Move already in progress. Aborting!");
            return PrepareMoveStatus::OTHER_COMMAND_IN_PROGRESS;
        }

        moveCommandName = commandNameForLogging;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            axes[nodeId].moveStatus = RobotConstants::MoveStatus::TASKED_WITH_MOVE;
        }
        isMAJInProgress = true;

        PrepareMoveComputationResult prepareResult = prepareMove(params, isAbsoluteMove);

        if (prepareResult.status == PrepareMoveStatus::NO_EFFECTIVE_MOTION)
        {
            MAJ_clearMoveStatusesAfterMoveCompletion();
            isMAJInProgress = false;
            moveCommandName = nullptr;
            return PrepareMoveStatus::NO_EFFECTIVE_MOTION;
        }

        if (prepareResult.status != PrepareMoveStatus::OK)
        {
            MAJ_clearMoveStatusesAfterMoveCompletion();
            isMAJInProgress = false;
            moveCommandName = nullptr;
            DBG_ERROR(DBG_GROUP_MOVE, prepareResult.reason);
            return prepareResult.status;
        }

        DBG_VERBOSE(DBG_GROUP_MOVE, "Prepared move successfully. Starting MAJ. isTriangularProfile=" + String(prepareResult.isTriangularProfile) +
                                        ", maxMovementAxisId=" + String(prepareResult.maxMovementAxisId) +
                                        ", maxMovementAbsSteps=" + String(prepareResult.maxMovementAbsSteps) +
                                        ", accelerationTimeSec=" + String(prepareResult.accelerationTimeSec, 4) +
                                        ", constantVelocityTimeSec=" + String(prepareResult.constantVelocityTimeSec, 4) +
                                        ", fullMovementTimeSec=" + String(prepareResult.fullMovementTimeSec, 4));

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            MAJ_start(nodeId);
        }

        return prepareResult.status;
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

    MoveControllerBase::PrepareMoveComputationResult MoveControllerBase::computePrepareMove(MoveInput &input)
    {
        String movesStr = "CPM ";
        for (int32_t steps : input.relativeMotions)
        {
            movesStr += String(steps) + " ";
        }
        DBG_VERBOSE(DBG_GROUP_MOVE, movesStr + " " + String(input.velocity, 4) + " " + String(input.acceleration, 4));

        PrepareMoveComputationResult result;
        result.status = PrepareMoveStatus::OK;

        if (input.velocity < 0 || RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_PERCENT < input.velocity)
        {
            result.status = PrepareMoveStatus::INVALID_SPEED;
            result.reason = "SPEED_OUT_OF_LIMITS";
            return result;
        }

        if (input.acceleration < 0 || RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_PERCENT < input.acceleration)
        {
            result.status = PrepareMoveStatus::INVALID_ACCELERATION;
            result.reason = "ACCELERATION_OUT_OF_LIMITS";
            return result;
        }

        double velocity = input.velocity;
        if (0 < velocity && velocity < RobotConstants::Control::MINIMUM_PROFILE_VELOCITY_IN_PERCENT)
        {
            velocity = RobotConstants::Control::MINIMUM_PROFILE_VELOCITY_IN_PERCENT;
        }

        double acceleration = input.acceleration;
        if (0 < acceleration && acceleration < RobotConstants::Control::MINIMUM_PROFILE_ACCELERATION_IN_PERCENT)
        {
            acceleration = RobotConstants::Control::MINIMUM_PROFILE_ACCELERATION_IN_PERCENT;
        }

        int32_t maxMovementAbs = 0;
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];
            axisResult.requestedMovement = true;
            if (input.relativeMotions[nodeId - 1] == 0)
            {
                axisResult.requestedMovement = false;
            }
            const int32_t relativeSteps = input.relativeMotions[nodeId - 1];
            axisResult.targetSteps = relativeSteps;

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
            return result;
        }

        if (velocity == 0)
        {
            result.status = PrepareMoveStatus::INVALID_SPEED;
            result.reason = "SPEED_IS_ZERO, BUT MOTION_REQUESTED";
            return result;
        }

        if (acceleration == 0)
        {
            result.status = PrepareMoveStatus::INVALID_ACCELERATION;
            result.reason = "ACCELERATION_IS_ZERO, BUT MOTION_REQUESTED";
            return result;
        }

        const double maxVelocityStepsPerSec = velocity * RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_STEPS_PER_SECOND;
        const double maxAccelerationStepsPerSec2 = acceleration * RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_STEPS_PER_SECOND2;

        const double dAccelForMaxVelocity = (maxVelocityStepsPerSec * maxVelocityStepsPerSec) / maxAccelerationStepsPerSec2;
        const double maxDistanceSteps = static_cast<double>(maxMovementAbs);
        if (!std::isfinite(dAccelForMaxVelocity) || !std::isfinite(maxDistanceSteps))
        {
            result.status = PrepareMoveStatus::INVALID_PROFILE;
            result.reason = "dAccelForMaxVelocity or maxDistanceSteps is not finite";
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
            result.status = PrepareMoveStatus::INVALID_TIMING;
            result.reason = "PROFILE_TIME_INVALID " + String(result.accelerationTimeSec, 3) + " " + String(result.constantVelocityTimeSec, 3) + " " + String(result.fullMovementTimeSec, 3);
            return result;
        }

        const double denominator = result.accelerationTimeSec + result.constantVelocityTimeSec;
        if (!std::isfinite(denominator) || denominator <= 0.0)
        {
            result.status = PrepareMoveStatus::INVALID_TIMING;
            result.reason = "PROFILE_DENOMINATOR_INVALID " + String(denominator, 3);
            return result;
        }

        bool hasEffectiveMotion = false;
        bool syncModelValid = true;

        DBG_INFO(DBG_GROUP_MOVE, "Result: accTime=" + String(result.accelerationTimeSec, 5) + ", constantVelTime=" + String(result.constantVelocityTimeSec, 5) + " " + String(result.fullMovementTimeSec, 5));
        DBG_INFO(DBG_GROUP_MOVE, "maxDistanceSteps=" + String(maxDistanceSteps, 2) + ", dAccelForMaxVelocity=" + String(dAccelForMaxVelocity, 2) + ", maxVelocityStepsPerSec=" + String(maxVelocityStepsPerSec, 2));
        DBG_INFO(DBG_GROUP_MOVE, "denominator=" + String(denominator, 3) + "; MaxMovementAbs=" + String(maxMovementAbs));

        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];
            const int32_t relativeAbsSteps = std::abs(axisResult.targetSteps);
            if (relativeAbsSteps == 0)
            {
                axisResult.profileVelocityRpm = 0;
                axisResult.profileAccelerationRpmPerSec = 0;
                continue;
            }
            else if (relativeAbsSteps == maxMovementAbs)
            {
                axisResult.profileVelocityRpm = std::round(velocity * RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_RPM);
                axisResult.profileAccelerationRpmPerSec = std::round(acceleration * RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S);
                continue;
            }

            hasEffectiveMotion = true;
            axisResult.velocityStepsPerSec = static_cast<double>(relativeAbsSteps) / denominator;
            axisResult.accelerationStepsPerSec2 = axisResult.velocityStepsPerSec / result.accelerationTimeSec;
            DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": velocityStepsPerSec=" + String(axisResult.velocityStepsPerSec) + "; accelerationStepsPerSec2=" + String(axisResult.accelerationStepsPerSec2));
            if (!std::isfinite(axisResult.velocityStepsPerSec) || axisResult.velocityStepsPerSec <= 0.0 ||
                !std::isfinite(axisResult.accelerationStepsPerSec2) || axisResult.accelerationStepsPerSec2 <= 0.0)
            {
                result.status = PrepareMoveStatus::INVALID_SPEED;
                result.reason =  "Axis " + String(nodeId) + ": velocityStepsPerSec or accelerationStepsPerSec2 infinite or non-positive";
                break;
            }
            
            const double velocityRpmDouble = Axis::stepsPerSecToMotorRPMDouble(axisResult.velocityStepsPerSec);
            const double accelerationRpmPerSecDouble = Axis::stepsPerSec2ToRPMPSDouble(axisResult.accelerationStepsPerSec2);

            if (!std::isfinite(velocityRpmDouble) || !std::isfinite(accelerationRpmPerSecDouble))
            {
                result.status = PrepareMoveStatus::INVALID_SPEED;
                result.reason =  "Axis " + String(nodeId) + ": velocityRpmDouble or accelerationRpmPerSecDouble infinite";
                break;
            }

            // DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(nodeId) + " velocityRpmDouble=" + String(velocityRpmDouble, 3) + ", accelerationRpmPerSecDouble=" + String(accelerationRpmPerSecDouble, 3));

            uint32_t profileVelocityRpm;
            if (axisResult.velocityStepsPerSec < RobotConstants::Control::MINIMUM_PROFILE_VELOCITY_IN_STEPS_PER_SEC)
            {
                profileVelocityRpm = RobotConstants::Control::MINIMUM_PROFILE_VELOCITY_IN_RPM;
            }
            else
            {
                profileVelocityRpm = static_cast<uint32_t>(std::round(velocityRpmDouble));
            }

            uint32_t profileAccelerationRpmPerSec;
            if (axisResult.accelerationStepsPerSec2 < RobotConstants::Control::MINIMUM_PROFILE_ACCELERATION_IN_STEPS_PER_SEC2)
            {
                profileAccelerationRpmPerSec = RobotConstants::Control::MINIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S;
            }
            else
            {
                profileAccelerationRpmPerSec = static_cast<uint32_t>(std::round(accelerationRpmPerSecDouble));
            }

            axisResult.profileVelocityRpm = std::min(profileVelocityRpm, RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_RPM);
            axisResult.profileAccelerationRpmPerSec = std::min(profileAccelerationRpmPerSec, RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S);
        }

        if (!hasEffectiveMotion)
        {
            result.status = PrepareMoveStatus::NO_EFFECTIVE_MOTION;
            result.reason = "NO_EFFECTIVE_MOTION";
        }

        return result;
    }

    MoveControllerBase::PrepareMoveComputationResult MoveControllerBase::prepareMove(const MoveParams<RobotConstants::Robot::AXES_COUNT> &params, bool isAbsoluteMove)
    {
        PrepareMoveComputationResult result;
        MoveInput input;

        String inputRelativeMotionsStr = "prepareMove: ";
        // Limits check and conversion to relative motions in steps
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            int32_t targetPositionInSteps = Axis::unitsToSteps(params.movementUnits[nodeId - 1]);
            if (!isAbsoluteMove)
            {
                targetPositionInSteps += axes.at(nodeId).getCurrentPositionInSteps().value();
            }

            if (!axes.at(nodeId).checkTargetPositionInStepsForLimits(targetPositionInSteps))
            {
                result.status = PrepareMoveStatus::INVALID_PROFILE_OUT_OF_LIMITS;
                result.reason = "Axis " + String(nodeId) + ": target position in steps " + String(targetPositionInSteps) + " is out of limits. lowLimit=" + String(axes.at(nodeId).lowLimitSteps) + ", highLimit=" + String(axes.at(nodeId).highLimitSteps);
                return result;
            }

            input.relativeMotions[nodeId - 1] = targetPositionInSteps - axes.at(nodeId).getCurrentPositionInSteps().value();
            inputRelativeMotionsStr += String(input.relativeMotions[nodeId - 1]) + " " + String(targetPositionInSteps) + " " + String(axes.at(nodeId).getCurrentPositionInSteps().value()) + "; ";
        }
        input.velocity = params.speed;
        input.acceleration = params.acceleration;

        DBG_VERBOSE(DBG_GROUP_MOVE, inputRelativeMotionsStr + "; velocity=" + String(input.velocity) + "; acceleration=" + String(input.acceleration));

        result = computePrepareMove(input);

        DBG_VERBOSE(DBG_GROUP_MOVE, "\nPrepareMoveComputationResult: status=" + prepareMoveStatusToString(result.status) + ", reason=" + result.reason);

        if (result.status == PrepareMoveStatus::OK || result.status == PrepareMoveStatus::NO_EFFECTIVE_MOTION)
        {

            for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
            {
                const PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];
                DBG_VERBOSE(DBG_GROUP_MOVE, "Axis " + String(nodeId) +
                                                ": requestedMovement=" + String(axisResult.requestedMovement) +
                                                ", targetSteps=" + String(axisResult.targetSteps) +
                                                ", profileVelocityRpm=" + String(axisResult.profileVelocityRpm) +
                                                ", profileAccelerationRpmPerSec=" + String(axisResult.profileAccelerationRpmPerSec, 4));
            }

            for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
            {
                Axis &axis = axes.at(nodeId);
                const PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];

                if (!axis.setTargetPositionInSteps(axes.at(nodeId).getCurrentPositionInSteps().value() + axisResult.targetSteps))
                {
                    axis.moveStatus = RobotConstants::MoveStatus::MOVE_PREPARATION_FAIL_OUT_OF_LIMITS;
                    result.status = PrepareMoveStatus::INVALID_PROFILE_OUT_OF_LIMITS;
                    result.reason = "Axis " + String(nodeId) + ": target position in steps " + String(axes.at(nodeId).getCurrentPositionInSteps().value() + axisResult.targetSteps) + " is out of limits. lowLimit=" + String(axis.lowLimitSteps) + ", highLimit=" + String(axis.highLimitSteps);
                    DBG_WARN(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": target position in steps " + String(axes.at(nodeId).getCurrentPositionInSteps().value() + axisResult.targetSteps) + " is out of limits. lowLimit=" + String(axis.lowLimitSteps) + ", highLimit=" + String(axis.highLimitSteps));
                    return result;
                }
                axis.setProfileVelocityInRPM(axisResult.profileVelocityRpm);
                axis.setProfileAccelerationInRPMPerSec(axisResult.profileAccelerationRpmPerSec);

                axis.moveStatus = RobotConstants::MoveStatus::MOVE_PREPARATION_SUCCESS;
            }

            DBG_INFO(DBG_GROUP_MOVE, "prepareMove status=" + prepareMoveStatusToString(result.status) + ", reason=" + result.reason + ", triangular=" + String(result.isTriangularProfile) + ", ta=" + String(result.accelerationTimeSec) + ", tc=" + String(result.constantVelocityTimeSec) + ", tt=" + String(result.fullMovementTimeSec));
            return result;
        }
        else
        {
            for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
            {
                axes.at(nodeId).moveStatus = RobotConstants::MoveStatus::MOVE_PREPARATION_FAIL;
            }
            DBG_ERROR(DBG_GROUP_MOVE, "prepareMove failed: status=" + prepareMoveStatusToString(result.status) + ", reason=" + result.reason);
            return result;
        }
    }

    // ============================ Protected methods end ===========================

    // ============================= Private methods =============================

    void MoveControllerBase::positionUpdate(uint8_t nodeId, int32_t position)
    {
        DBG_VERBOSE(DBG_GROUP_CANOPEN, "Position update from node " + String(nodeId) + ": " + String(position));
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

            if ((now - lastHb) > RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && axis.status != RobotConstants::AxisStatus::NOT_ALIVE)
            {
                DBG_ERROR(DBG_GROUP_HEARTBEAT, "==== Heartbeat timeout for Axis " + String(nodeId) + " ====");
                axis.status = RobotConstants::AxisStatus::NOT_ALIVE;
            }
            else if ((now - lastHb) <= RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && axis.status == RobotConstants::AxisStatus::NOT_ALIVE)
            {
                DBG_ERROR(DBG_GROUP_HEARTBEAT, "==== Heartbeat restored for Axis " + String(nodeId) + " ====");

                axis.status = RobotConstants::AxisStatus::ALIVE_BUT_NOT_INITIALIZED;

                FAL_start(axis.nodeId);
            }
        }
    }

    void MoveControllerBase::tick_checkZEITimeouts()
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.initStatus == RobotConstants::InitStatus::ZEI_ONGOING && axis.status == RobotConstants::AxisStatus::NOT_ALIVE)
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
            if (axes[nodeId].status == RobotConstants::AxisStatus::ALIVE)
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
            if (axis.moveStatus == RobotConstants::MoveStatus::MOVING && axis.status == RobotConstants::AxisStatus::NOT_ALIVE)
            {
                DBG_WARN(DBG_GROUP_MOVE, "MAJ failed for Axis " + String(nodeId) + ": Heartbeat timeout");
                axis.moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
                MAJ_finalResult();
            }

            uint32_t now = millis();
            if (now - axis.lastRequestedStatusWord > 100 && axis.moveStatus == RobotConstants::MoveStatus::MOVING)
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
        canOpen->set_callback_x6081_profileVelocity([this](uint8_t callbackNodeId, bool success)
                                                    { this->ZEI_AfterWriteTo_0x6081(callbackNodeId, success); }, nodeId);

        // Step 4
        bool successSend = canOpen->send_x6081_profileVelocity(nodeId,
                                                               0x0000);

        // Step 5
        if (!ZEI_checkResponseStatus(nodeId, successSend,
                                     "ZEI: Failed to send profile velocity <- 0x0000"))
        {
            // Step 6
            canOpen->set_callback_x6081_profileVelocity(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZEI_AfterWriteTo_0x6081(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6081_profileVelocity(nullptr, nodeId);
        // Step 2
        if (!ZEI_checkResponseStatus(nodeId, success,
                                     "ZEI: Failed to write profile velocity to 0x6081"))
        {
            return;
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

        String commandReply = RobotConstants::Commands::ZERO_INITIALIZE + " " + status;
        if (status == RobotConstants::Status::COMMAND_PARTIAL_FAIL)
        {
            commandReply += " " + successfullAxes + " | " + failedAxes;
        }
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
        }
        else
        {
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
                                                               axes[nodeId].getProfileVelocityInRPM().value());
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
                                                                   axes[nodeId].getProfileAccelerationInRPMPerSec().value());
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
                                               axes[nodeId].getTargetPositionInSteps().value());
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
        axes[nodeId].moveStatus = RobotConstants::MoveStatus::READY_TO_MOVE;
        MAJ_SYNCFunnel();
    }

    void MoveControllerBase::MAJ_SYNCFunnel()
    {

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            RobotConstants::MoveStatus status = axes[nodeId].moveStatus;
            if (status == RobotConstants::MoveStatus::MOVE_PREPARATION_SUCCESS)
            {

                return; // Not all axes are ready yet
            }
        }

        // All axes are ready, send SYNC
        DBG_INFO(DBG_GROUP_MOVE, "MAJ_SYNCFunnel: All axes are ready. Sending SYNC and starting movement.");
        canOpen->sendSYNC();
        delay(10); // Check for different
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].moveStatus == RobotConstants::MoveStatus::READY_TO_MOVE)
            {
                axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVING;
                MAJ_requestStatusWord(nodeId); // Request status immediately after sending SYNC to minimize the delay before we get the first status update
                delay(10);
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

    void MoveControllerBase::MAJ_statusWordCallback(uint8_t nodeId, bool success, uint16_t statusWord)
    {
        // DBG_INFO(DBG_GROUP_MOVE, "MAJ_statusWordCallback called for node " + String(nodeId) + " with success=" + String(success) + " and statusWord=0x" + String(statusWord, HEX));
        //  Step 1
        canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to read statusword for Axis " + String(nodeId)))
        {
            return;
        }
        // Step 3 (Data processing)
        // DBG_INFO(DBG_GROUP_MOVE, "MAJ Status Word from node " + String(nodeId) + ": 0x" + String(statusWord, HEX));
        if (MAJ_checkTargetPositionReached(statusWord))
        {
            // DBG_INFO(DBG_GROUP_MOVE, "MAJ Target position reached for Axis " + String(nodeId));
            axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_SUCCESS;
            DBG_INFO(DBG_GROUP_MOVE, "MAJ Movement finished for Axis " + String(nodeId));
            MAJ_finalResult();
            return;
        }
    }

    void MoveControllerBase::MAJ_finalResult()
    {
        String successfullAxes = "";
        String failedAxes = "";
        String unknownErrorAxes = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].moveStatus == RobotConstants::MoveStatus::MOVING || axes[nodeId].moveStatus == RobotConstants::MoveStatus::MOVE_PREPARATION_SUCCESS || axes[nodeId].moveStatus == RobotConstants::MoveStatus::READY_TO_MOVE)
            {
                DBG_INFO(DBG_GROUP_MOVE, "Still going for Axis " + String(nodeId));
                return; // Still ongoing for some axes
            }
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.moveStatus == RobotConstants::MoveStatus::MOVE_SUCCESS)
            {
                successfullAxes += String(nodeId) + " ";
            }
            else if (axis.moveStatus == RobotConstants::MoveStatus::MOVE_FAIL)
            {
                failedAxes += String(nodeId) + " ";
            }
            else
            {
                DBG_ERROR(DBG_GROUP_MOVE, "MAJ_finalResult called for Axis " + String(nodeId) + " with invalid status: " + String(axis.moveStatus));
                unknownErrorAxes += String(nodeId) + " ";
            }
        }

        MAJ_clearMoveStatusesAfterMoveCompletion();

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

        String commandReply = (moveCommandName == nullptr ? RobotConstants::Status::UNKNOWN_ERROR : *moveCommandName) + " " + status + " " + successfullAxes + "|" + failedAxes + "|" + unknownErrorAxes;
        addDataToOutQueue(commandReply);
        isMAJInProgress = false;
        moveCommandName = nullptr;
    }

    bool MoveControllerBase::MAJ_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "MAJ Failed for Axis " + String(nodeId) + ": " + errorMessage);
            axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
            MAJ_finalResult();
        }
        if (axes[nodeId].status == RobotConstants::AxisStatus::NOT_ALIVE)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "MAJ Failed for Axis " + String(nodeId) + ": Axis is not alive (heartbeat timeout)");
            axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
            MAJ_finalResult();
        }

        return success;
    }

    bool MoveControllerBase::MAJ_checkTargetPositionReached(uint16_t statusWord)
    {
        // Check 10-th bit (0-indexed) of the status word (0x0400) to determine if the movement is finished
        return (statusWord & 0b10000000000) != 0;
    }
    // ======== MAJ Sequence End ========

    // ======== Fixate Axis after restoring life ========
    void MoveControllerBase::FAL_start(uint8_t nodeId)
    {
        // Step 4
        canOpen->set_callback_x6040_controlword([this](uint8_t cbNodeId, bool cbSuccess)
                                                { this->FAL_afterWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);

        // Step 5
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x000F);
        // Step 6
        if (!successSend)
        {
            DBG_ERROR(DBG_GROUP_HEARTBEAT, "Axis " + String(nodeId) + ": send of 0xF to 0x6040 failed. Setting status to NOT_ALIVE");
            axes.at(nodeId).status = RobotConstants::AxisStatus::NOT_ALIVE;
            // Step 7
            canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::FAL_afterWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_HEARTBEAT, "Axis " + String(nodeId) + ": write of 0xF to 0x6040 failed. Setting status to NOT_ALIVE");
            axes.at(nodeId).status = RobotConstants::AxisStatus::NOT_ALIVE;
        }
        else
        {
            DBG_ERROR(DBG_GROUP_HEARTBEAT, "Axis " + String(nodeId) + ": write of 0xF to 0x6040 success. Setting status to ALIVE");
            axes.at(nodeId).status = RobotConstants::AxisStatus::ALIVE;
        }
    }

    // ======== End of Fixate Axis after restoring life ========

    String MoveControllerBase::prepareMoveStatusToString(PrepareMoveStatus status)
    {
        switch (status)
        {
        case PrepareMoveStatus::OK:
            return "OK";
        case PrepareMoveStatus::NO_EFFECTIVE_MOTION:
            return "NM";
        case PrepareMoveStatus::INVALID_SPEED:
            return "IS";
        case PrepareMoveStatus::INVALID_ACCELERATION:
            return "IA";
        case PrepareMoveStatus::INVALID_PROFILE:
            return "IF";
        case PrepareMoveStatus::INVALID_PROFILE_OUT_OF_LIMITS:
            return "IL";
        case PrepareMoveStatus::INVALID_TIMING:
            return "IT";
        case PrepareMoveStatus::NOT_INITIALIZED:
            return "NI";
        case PrepareMoveStatus::OTHER_COMMAND_IN_PROGRESS:
            return "OC";
        default:
            return "XE";
        }
    }

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