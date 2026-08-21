#include <algorithm>
#include <cmath>
#include <utility>

#include "MoveControllerBase.h"
#include "Arduino.h"
#include "Debug.h"

#include <EEPROM.h>

namespace StepDirController
{
    // ============================= Public methods =============================

    String MoveControllerBase::requestStatus()
    {
        String status = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes.at(nodeId);
            status += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) + String(axis.status) + " ";
        }
        return status;
    }

    std::optional<int32_t> MoveControllerBase::axisPosition(uint8_t nodeId)
    {
        if (nodeId < 1 || RobotConstants::Robot::AXES_COUNT < nodeId)
        {
            return std::nullopt;
        }
        return axes.at(nodeId).getCurrentPositionInSteps().value();
    }

    ParamsStatusStruct MoveControllerBase::start(CanOpen *canOpen,
                                                 uint8_t axesCnt, 
                                                 std::unique_ptr<KinematicSolver<RobotConstants::Robot::AXES_COUNT>> solver_ptr, 
                                                 uint8_t *nodesToInvert, 
                                                 uint8_t nodesToInvertCnt)
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
        this->solver = std::move(solver_ptr);

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

        majMoveTimeoutMs = RobotConstants::Control::MAJ_MOVE_TIMEOUT_MS;
        initialized = true;

        return status;
    }

    // void MoveControllerBase::startZeroInitializationAllAxes()
    // {
    //     DBG_VERBOSE(DBG_GROUP_ZOE, "Start ZOE for all axes");
    //     zeroInitializeSingleAxis = false;
    //     for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
    //     {
    //         startZeroInitializationSingleAxis(nodeId);
    //     }
    // }

    void MoveControllerBase::startZeroInitializationSingleAxis(uint8_t nodeId)
    {
        ZOE_start(nodeId);
    }

    MoveControllerBase::PrepareMoveStatus MoveControllerBase::move(MoveParams<RobotConstants::Robot::AXES_COUNT> params, bool isAbsoluteMove, const String *commandNameForLogging)
    {
        if (!initialized)
        {
            DBG_VERBOSE(DBG_GROUP_MOVE, "MoveControllerBase::move failed. Not initialized");
            return PrepareMoveStatus::FAIL;
        }
        if (isMAJInProgress)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "Move already in progress. Aborting!");
            return PrepareMoveStatus::OTHER_COMMAND_IN_PROGRESS;
        }

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes.at(nodeId).status != RobotConstants::AxisStatus::ALIVE)
            {
                DBG_ERROR(DBG_GROUP_MOVE, "Motor " + String(nodeId) + " not alive");
                return PrepareMoveStatus::FAIL;
            }
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
            DBG_INFO(DBG_GROUP_MOVE, "HEllo, it is bad");
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

        majMoveStartMs = millis();
        majMoveTimedOut = false;

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            MAJ_start(nodeId);
        }

        return prepareResult.status;
    }

    MoveControllerBase::PrepareMoveStatus MoveControllerBase::moveCartesian(MoveCartesianParams targetPosition, bool isAbsoluteMove, const String *commandNameForLogging)
    {
        MoveParams<RobotConstants::Robot::AXES_COUNT> params;
        params.move = targetPosition.move;

        JointAngles<RobotConstants::Robot::AXES_COUNT> angles = solver->ik(targetPosition.position);

        DBG_VERBOSE(DBG_GROUP_MAC, "My angles: " + 
                            String(angles.angles[0], 3) + " " +
                            String(angles.angles[1], 3) + " " +
                            String(angles.angles[2], 3) + " " +
                            String(angles.angles[3], 3) + " " + String(angles.isValid));

        if (!angles.isValid)
        {
            return PrepareMoveStatus::OUT_OF_LIMITS;
        }

        params.angles = angles;

        DBG_VERBOSE(DBG_GROUP_MAC, "Params.angles: " + 
                                    String(params.angles.angles[0], 3) + " " +
                                    String(params.angles.angles[1], 3) + " " +
                                    String(params.angles.angles[2], 3) + " " +
                                    String(params.angles.angles[3], 3));
        
        //return PrepareMoveStatus::OUT_OF_LIMITS;
        return move(params, true, commandNameForLogging);
    }

    void MoveControllerBase::tick_100()
    {
        if (!initialized)
        {
            return;
        }
        tick_checkMajMoveTimeout();
        tick_pollMajPositionDuringMove();
    }

    void MoveControllerBase::tick_500()
    {
        if (!initialized)
        {
            return;
        }
        tick_checkTimeouts();
        tick_checkZOETimeouts();
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
        double velocity, acceleration;
        if (input.velocity < 0)
        {
            result.status = PrepareMoveStatus::INVALID_PARAMS;
            result.reason = "SPEED_OUT_OF_LIMITS";
            return result;
        }
        else if (RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_PERCENT < input.velocity)
        {
            velocity = RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_PERCENT;
        }
        else
        {
            velocity = input.velocity;
        }

        if (input.acceleration < 0)
        {
            result.status = PrepareMoveStatus::INVALID_PARAMS;
            result.reason = "ACCELERATION_OUT_OF_LIMITS";
            return result;
        }
        else if (RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_PERCENT < input.acceleration)
        {
            acceleration = RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_PERCENT;
        }
        else
        {
            acceleration = input.acceleration;
        }

        if (0 < velocity && velocity < RobotConstants::Control::MINIMUM_PROFILE_VELOCITY_IN_PERCENT)
        {
            velocity = RobotConstants::Control::MINIMUM_PROFILE_VELOCITY_IN_PERCENT;
        }

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
            result.status = PrepareMoveStatus::INVALID_PARAMS;
            result.reason = "SPEED_IS_ZERO, BUT MOTION_REQUESTED";
            return result;
        }

        if (acceleration == 0)
        {
            result.status = PrepareMoveStatus::INVALID_PARAMS;
            result.reason = "ACCELERATION_IS_ZERO, BUT MOTION_REQUESTED";
            return result;
        }

        const double maxVelocityStepsPerSec = velocity * RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_STEPS_PER_SECOND;
        const double maxAccelerationStepsPerSec2 = acceleration * RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_STEPS_PER_SECOND2;

        const double dAccelForMaxVelocity = (maxVelocityStepsPerSec * maxVelocityStepsPerSec) / maxAccelerationStepsPerSec2;
        const double maxDistanceSteps = static_cast<double>(maxMovementAbs);
        // if (!std::isfinite(dAccelForMaxVelocity) || !std::isfinite(maxDistanceSteps))
        // {
        //     result.status = PrepareMoveStatus::INVALID_PROFILE;
        //     result.reason = "dAccelForMaxVelocity or maxDistanceSteps is not finite";
        //     return result;
        // }

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
            result.status = PrepareMoveStatus::FAIL;
            result.reason = "PROFILE_TIME_INVALID " + String(result.accelerationTimeSec, 3) + " " + String(result.constantVelocityTimeSec, 3) + " " + String(result.fullMovementTimeSec, 3);
            return result;
        }

        const double denominator = result.accelerationTimeSec + result.constantVelocityTimeSec;
        if (!std::isfinite(denominator) || denominator <= 0.0)
        {
            result.status = PrepareMoveStatus::FAIL;
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
            // const int32_t relativeAbsSteps = std::abs(axisResult.targetSteps);
            const double relativeAbsSteps = std::abs(static_cast<double>(axisResult.targetSteps));
            if (relativeAbsSteps == 0)
            {
                // axisResult.profileVelocityRpm = 0;
                // axisResult.profileAccelerationRpmPerSec = 0;
                continue;
            }
            else if (relativeAbsSteps == maxMovementAbs)
            {
                axisResult.profileVelocityRpm = std::round(velocity * RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_RPM);
                axisResult.profileAccelerationRpmPerSec = std::round(acceleration * RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S);
                hasEffectiveMotion = true;
                continue;
            }

            hasEffectiveMotion = true;
            // axisResult.velocityStepsPerSec = static_cast<double>(relativeAbsSteps) / denominator;
            axisResult.velocityStepsPerSec = relativeAbsSteps / denominator;
            axisResult.accelerationStepsPerSec2 = axisResult.velocityStepsPerSec / result.accelerationTimeSec;
            // if (!std::isfinite(axisResult.velocityStepsPerSec) || axisResult.velocityStepsPerSec <= 0.0 ||
            //     !std::isfinite(axisResult.accelerationStepsPerSec2) || axisResult.accelerationStepsPerSec2 <= 0.0)
            // {
            //     result.status = PrepareMoveStatus::INVALID_PROFILE;
            //     result.reason = "Axis " + String(nodeId) + ": velocityStepsPerSec or accelerationStepsPerSec2 infinite or non-positive";
            //     break;
            // }

            const double velocityRpmDouble = Axis::stepsPerSecToMotorRPMDouble(axisResult.velocityStepsPerSec);
            const double accelerationRpmPerSecDouble = Axis::stepsPerSec2ToRPMPSDouble(axisResult.accelerationStepsPerSec2);

            DBG_INFO(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": velocityStepsPerSec=" + String(axisResult.velocityStepsPerSec) +
                                         "; accelerationStepsPerSec2=" + String(axisResult.accelerationStepsPerSec2) +
                                         "; velocityRpmDouble=" + String(velocityRpmDouble, 3) +
                                         "; accelerationRpmPerSecDouble=" + String(accelerationRpmPerSecDouble, 3) + " " + String(hasEffectiveMotion));

            // if (!std::isfinite(velocityRpmDouble) || !std::isfinite(accelerationRpmPerSecDouble))
            // {
            //     result.status = PrepareMoveStatus::INVALID_PROFILE;
            //     result.reason = "Axis " + String(nodeId) + ": velocityRpmDouble or accelerationRpmPerSecDouble infinite";
            //     break;
            // }

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

            if (result.isTriangularProfile)
            {
                // axisResult.profileVelocityRpm = 0;
            }
            else
            {
                // axisResult.profileVelocityRpm = std::min(profileVelocityRpm, RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_RPM);
            }
            axisResult.profileVelocityRpm = std::min(profileVelocityRpm, RobotConstants::Control::MAXIMUM_PROFILE_VELOCITY_IN_RPM);
            axisResult.profileAccelerationRpmPerSec = std::min(profileAccelerationRpmPerSec, RobotConstants::Control::MAXIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S);
        }

        String t = "";
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            t += "Axis " + String(nodeId) + " " + String(result.axes[nodeId - 1].profileAccelerationRpmPerSec) + " ";
        }
        DBG_WARN(DBG_GROUP_MOVE, "profileAccelerationRpmPerSec: " + t);

        if (!hasEffectiveMotion)
        {
            DBG_INFO(DBG_GROUP_MOVE, "!hasEffectiveMotion");
            result.status = PrepareMoveStatus::NO_EFFECTIVE_MOTION;
            result.reason = "NO_EFFECTIVE_MOTION";
        }
        DBG_INFO(DBG_GROUP_MOVE, "Return from computePrepareMove");
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
            int32_t targetPositionInSteps = Axis::degreesToSteps(params.angles.angles[nodeId - 1]);
            if (!isAbsoluteMove)
            {
                targetPositionInSteps += axes.at(nodeId).getCurrentPositionInSteps().value();
            }

            if (!axes.at(nodeId).checkTargetPositionInStepsForLimits(targetPositionInSteps))
            {
                result.status = PrepareMoveStatus::OUT_OF_LIMITS;
                result.reason = "Axis " + String(nodeId) + ": target position in steps " + String(targetPositionInSteps) + " is out of limits. lowLimit=" + String(axes.at(nodeId).lowLimitSteps) + ", highLimit=" + String(axes.at(nodeId).highLimitSteps);
                return result;
            }

            input.relativeMotions[nodeId - 1] = targetPositionInSteps - axes.at(nodeId).getCurrentPositionInSteps().value();
            inputRelativeMotionsStr += String(input.relativeMotions[nodeId - 1]) + " " + String(targetPositionInSteps) + " " + String(axes.at(nodeId).getCurrentPositionInSteps().value()) + "; ";
        }
        input.velocity = params.move.speed;
        input.acceleration = params.move.acceleration;

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
                    axis.moveStatus = RobotConstants::MoveStatus::MOVE_PREPARATION_FAIL;
                    result.status = PrepareMoveStatus::OUT_OF_LIMITS;
                    result.reason = "Axis " + String(nodeId) + ": target position in steps " + String(axes.at(nodeId).getCurrentPositionInSteps().value() + axisResult.targetSteps) + " is out of limits. lowLimit=" + String(axis.lowLimitSteps) + ", highLimit=" + String(axis.highLimitSteps);
                    DBG_WARN(DBG_GROUP_MOVE, "Axis " + String(nodeId) + ": target position in steps " + String(axes.at(nodeId).getCurrentPositionInSteps().value() + axisResult.targetSteps) + " is out of limits. lowLimit=" + String(axis.lowLimitSteps) + ", highLimit=" + String(axis.highLimitSteps));
                    return result;
                }
                axis.setProfileVelocityInRPM(axisResult.profileVelocityRpm);
                axis.setProfileAccelerationInRPMPerSec(axisResult.profileAccelerationRpmPerSec);
                // if (nodeId == 1 || nodeId == 2 || nodeId == 3){
                //     Serial2.println("setting profile acceleration " + String(nodeId) + " " + String(axisResult.profileAccelerationRpmPerSec) );
                // }
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

    void MoveControllerBase::tick_checkZOETimeouts()
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.initStatus == RobotConstants::InitStatus::ZOE_ONGOING && axis.status == RobotConstants::AxisStatus::NOT_ALIVE)
            {
                DBG_WARN(DBG_GROUP_ZOE, "Zero Initialization failed for Axis " + String(nodeId) + ": Heartbeat timeout");
                axis.initStatus = RobotConstants::InitStatus::ZOE_FAILED;
                ZOE_finalResult();
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

    void MoveControllerBase::tick_checkMajMoveTimeout()
    {
        if (!isMAJInProgress || majMoveStartMs == 0)
        {
            return;
        }

        const uint32_t elapsed = millis() - majMoveStartMs;
        if (elapsed < majMoveTimeoutMs)
        {
            return;
        }

        DBG_WARN(DBG_GROUP_MOVE, "MAJ: timeout after " + String(majMoveTimeoutMs) + " ms; failing all axes");

        majMoveTimedOut = true;

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
        }

        MAJ_finalResult();
    }

    void MoveControllerBase::tick_pollMajPositionDuringMove()
    {
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if (axis.moveStatus == RobotConstants::MoveStatus::MOVING)
            {
                if (axis.status == RobotConstants::AxisStatus::NOT_ALIVE)
                {
                    DBG_WARN(DBG_GROUP_MOVE, "MAJ failed for Axis " + String(nodeId) + ": Heartbeat timeout");
                    axis.moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
                    MAJ_finalResult();
                }
                const uint32_t now = millis();
                if (now - axis.lastMajPositionPollMs > 100)
                {
                    MAJ_requestPositionPollDuringMove(nodeId);
                }
            }
        }
    }

    // ======== Timer functions end ========

    // ======== ZOE Sequence ========
    void MoveControllerBase::ZOE_start(uint8_t nodeId)
    {
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZOE_ONGOING;
        // if (zeroInitializeSingleAxis)
        // {
        axisToInitialize = nodeId;
        // }

        // Step 3
        canOpen->set_callback_x6081_profileVelocity([this](uint8_t callbackNodeId, bool success)
                                                    { this->ZOE_AfterWriteTo_0x6081(callbackNodeId, success); }, nodeId);

        // Step 4
        bool successSend = canOpen->send_x6081_profileVelocity(nodeId,
                                                               0x0000);

        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send profile velocity <- 0x0000"))
        {
            // Step 6
            canOpen->set_callback_x6081_profileVelocity(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZOE_AfterWriteTo_0x6081(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6081_profileVelocity(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write profile velocity to 0x6081"))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x6040_controlword([this](uint8_t callbackNodeId, bool success)
                                                { this->ZOE_AfterFirstWriteTo_0x6040(callbackNodeId, success); }, nodeId);

        // Step 4
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x0000);

        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send control word <- 0x0000"))
        {
            // Step 6
            canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZOE_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write 0x0000 to 0x6040"))
        {
            return;
        }
        // Step 3
        canOpen->set_callback_x260A_electronicGearMolecules([this](uint8_t cbNodeId, bool cbSuccess)
                                                            { this->ZOE_AfterFirstWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA66);
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send electronic gear molecules <- 0xEA66"))
        {
            // Step 6
            canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZOE_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write 0xEA66 to 0x260A"))
        {
            return;
        }
        // Step 3
        canOpen->set_callback_x260A_electronicGearMolecules([this](uint8_t cbNodeId, bool cbSuccess)
                                                            { this->ZOE_AfterSecondWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA70);
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send electronic gear molecules <- 0xEA70"))
        {
            // Step 6
            canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZOE_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x260A_electronicGearMolecules(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write 0xEA70 to 0x260A"))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x6040_controlword([this](uint8_t cbNodeId, bool cbSuccess)
                                                { this->ZOE_AfterSecondWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        delay(200);
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x000F);
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send control word <- 0x000F"))
        {
            // Step 6
            canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::ZOE_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x6040_controlword(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write 0x000F to 0x6040"))
        {
            return;
        }
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZOE_FINISHED;
        ZOE_finalResult();
    }

    void MoveControllerBase::ZOE_finalResult()
    {
        String status;
        Axis &axis = axes[axisToInitialize];
        if (axis.initStatus == RobotConstants::InitStatus::ZOE_FINISHED)
        {
            status = RobotConstants::Result::OK;
        }
        else if (axis.initStatus == RobotConstants::InitStatus::ZOE_FAILED)
        {
            status = RobotConstants::Result::FAIL;
        }
        else
        {
            status = RobotConstants::Result::ERROR_UNKNOWN;
        }

        String commandReply = RobotConstants::Commands::ZERO_OUT_ENCODER + " " + status + " AN" + String(axisToInitialize);
        addDataToOutQueue(commandReply);

        axisToInitialize = 0;
        return;

        // String successfullAxes = "";
        // String failedAxes = "";
        // for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        // {

        //     if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZOE_ONGOING)
        //     {
        //         return; // Still ongoing for some axes
        //     }
        //     else if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZOE_FINISHED)
        //     {
        //         successfullAxes += String(nodeId) + " ";
        //     }
        //     else if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZOE_FAILED)
        //     {
        //         failedAxes += String(nodeId) + " ";
        //     }
        // }

        // String status;
        // if (failedAxes.length() > 0)
        // {
        //     status = RobotConstants::Result::FAIL;
        // }
        // else if (successfullAxes.length() > 0)
        // {
        //     status = RobotConstants::Result::OK;
        // }

        // zeroInitializeSingleAxis = true; // Reset to default for the next ZOE command

        // String commandReply = RobotConstants::Commands::ZERO_OUT_ENCODER + " " + status;
        // // if (status == RobotConstants::Result::FAIL)
        // // {
        // //     commandReply += " " + successfullAxes + " | " + failedAxes;
        // // }
        // addDataToOutQueue(commandReply);
    }

    bool MoveControllerBase::ZOE_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_ZOE, "ZOE Failed for Axis " + String(nodeId) + ": " + errorMessage);
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZOE_FAILED;
            ZOE_finalResult();
        }
        return success;
    }
    // ======== ZOE Sequence End ========

    // ======== MAJ Sequence ========

    void MoveControllerBase::MAJ_start(uint8_t nodeId)
    {
        if (axes.at(nodeId).status != RobotConstants::AxisStatus::ALIVE)
        {
            MAJ_checkResponseStatus(nodeId, false, "MAJ: Axis not alive!");
            return;
        }
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
        DBG_VERBOSE(DBG_GROUP_MOVE, "afterRequestOf_0x6040");
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
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_afterWriteTo_0x6040");
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
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_setTargetVelocity");
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
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_afterWriteTo_0x6081");
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
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_afterWriteTo_0x6083");
        canOpen->set_callback_x6083_profileAcceleration(nullptr, nodeId);
        // Step 2
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to set profile acceleration (0x6083) for Axis " + String(nodeId)))
        {
            return;
        }
        // delay(1000);
        //  Step 3
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
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_TPDO4");
        canOpen->set_callback_TPDO4(nullptr, nodeId);
        // Step 2
        // if (!MAJ_checkResponseStatus(nodeId, success,
        //                              "MAJ: Failed to set profile acceleration (0x6083) for Axis " + String(nodeId)))
        // {
        //     return;
        // }

        // Step 3 (Data processing)
        axes[nodeId].moveStatus = RobotConstants::MoveStatus::READY_TO_MOVE;
        MAJ_SYNCFunnel();
    }

    void MoveControllerBase::MAJ_SYNCFunnel()
    {
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_SYNCFunnel");
        bool containsFailedAxes = false;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            RobotConstants::MoveStatus status = axes[nodeId].moveStatus;
            if (status == RobotConstants::MoveStatus::MOVE_PREPARATION_SUCCESS)
            {

                return; // Not all axes are ready yet
            }
            else if (status == RobotConstants::MoveStatus::MOVE_FAIL)
            {
                containsFailedAxes = true;
            }
        }
        if (containsFailedAxes)
        {
            for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
            {
                axes.at(nodeId).moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
            }
            MAJ_finalResult();
            return;
        }

        // All axes are ready, send SYNC
        // DBG_INFO(DBG_GROUP_MOVE, "MAJ_SYNCFunnel: All axes are ready. Sending SYNC and starting movement.");
        canOpen->sendSYNC();
        DBG_VERBOSE(DBG_GROUP_MOVE, "sent sync");
        delay(10); // Check for different.
        /* Small delay after SYNC so the axis has applied the new setpoint before the first position poll. */
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].moveStatus == RobotConstants::MoveStatus::READY_TO_MOVE)
            {
                axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVING;
                MAJ_requestPositionPollDuringMove(nodeId);
                delay(10);
            }
        }
    }

    void MoveControllerBase::MAJ_requestPositionPollDuringMove(uint8_t nodeId)
    {
        DBG_VERBOSE(DBG_GROUP_MOVE, "MAJ_requestPositionPollDuringMove");
        canOpen->set_callback_x6064_positionActualValue([this](uint8_t cbNodeId, bool cbSuccess, int32_t positionActualValue)
                                                        { this->MAJ_positionPollDuringMove(cbNodeId, cbSuccess, positionActualValue); }, nodeId);

        const bool successSend = canOpen->sendSDORead(nodeId,
                                                      RobotConstants::ODIndices::POSITION_ACTUAL_VALUE,
                                                      RobotConstants::ODIndices::DEFAULT_SUBINDEX);
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send position poll for Axis " + String(nodeId)))
        {
            setRegularPositionActualValueCallback(nodeId);
        }

        axes[nodeId].lastMajPositionPollMs = millis();
    }

    void MoveControllerBase::MAJ_requestFaultStatusWordAfterPositionPoll(uint8_t nodeId)
    {
        if (axes[nodeId].moveStatus != RobotConstants::MoveStatus::MOVING)
        {
            return;
        }

        canOpen->set_callback_read_x6041_statusword([this](uint8_t cbNodeId, bool success, uint16_t statusWord)
                                                     { this->MAJ_faultOnlyStatusWordCallback(cbNodeId, success, statusWord); }, nodeId);

        const bool successSend = canOpen->sendSDORead(nodeId,
                                                      RobotConstants::ODIndices::STATUSWORD,
                                                      RobotConstants::ODIndices::DEFAULT_SUBINDEX);
        if (!MAJ_checkResponseStatus(nodeId, successSend,
                                     "MAJ: Failed to send fault statusword request for Axis " + String(nodeId)))
        {
            canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
        }
    }

    void MoveControllerBase::MAJ_faultOnlyStatusWordCallback(uint8_t nodeId, bool success, uint16_t statusWord)
    {
        canOpen->set_callback_read_x6041_statusword(nullptr, nodeId);
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to read fault statusword for Axis " + String(nodeId)))
        {
            return;
        }

        if (axes[nodeId].moveStatus != RobotConstants::MoveStatus::MOVING)
        {
            return;
        }

        if ((statusWord & 0x0008) != 0)
        {
            DBG_WARN(DBG_GROUP_MOVE, "MAJ: CiA402 fault bit set on Axis " + String(nodeId) + " statusword=0x" + String(statusWord, HEX));
            axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
            MAJ_finalResult();
            return;
        }

        // Throttle next host position poll until this position+fault cycle has finished (avoids overlapping SDOs).
        axes[nodeId].lastMajPositionPollMs = millis();
    }

    void MoveControllerBase::MAJ_positionPollDuringMove(uint8_t nodeId, bool success, int32_t positionActualValue)
    {
        if (axes[nodeId].moveStatus != RobotConstants::MoveStatus::MOVING)
        {
            setRegularPositionActualValueCallback(nodeId);
            return;
        }

        if (!success)
        {
            setRegularPositionActualValueCallback(nodeId);
            MAJ_checkResponseStatus(nodeId, false,
                                     "MAJ: Failed to read position poll for Axis " + String(nodeId));
            return;
        }

        if (!axes[nodeId].isPositionWithinMajTolerance(positionActualValue))
        {
            setRegularPositionActualValueCallback(nodeId);
            MAJ_requestFaultStatusWordAfterPositionPoll(nodeId);
            return;
        }

        MAJ_finishAxisAfterVerifiedPositionRead(nodeId, success, positionActualValue);
    }

    void MoveControllerBase::MAJ_finishAxisAfterVerifiedPositionRead(uint8_t nodeId, bool success, int32_t positionActualValue)
    {
        setRegularPositionActualValueCallback(nodeId);
        if (!MAJ_checkResponseStatus(nodeId, success,
                                     "MAJ: Failed to get the position of the Axis " + String(nodeId) + " after move finished"))
        {
            return;
        }

        axes[nodeId].setCurrentPositionInSteps(positionActualValue);
        axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_SUCCESS;
        DBG_INFO(DBG_GROUP_MOVE, "MAJ Movement finished for Axis (position updated) " + String(nodeId));
        MAJ_finalResult();
    }

    void MoveControllerBase::MAJ_finalResult()
    {
        String successfullAxes = "";
        String failedAxes = "";
        String unknownErrorAxes = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].moveStatus == RobotConstants::MoveStatus::MOVING ||
                axes[nodeId].moveStatus == RobotConstants::MoveStatus::MOVE_PREPARATION_SUCCESS ||
                axes[nodeId].moveStatus == RobotConstants::MoveStatus::READY_TO_MOVE)
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
        if ((failedAxes.length() + unknownErrorAxes.length()) > 0 && successfullAxes.length() > 0)
        {
            status = RobotConstants::Result::FAIL;
        }
        else if ((failedAxes.length() + unknownErrorAxes.length()) > 0)
        {
            status = RobotConstants::Result::FAIL;
        }
        else if (successfullAxes.length() > 0)
        {
            status = RobotConstants::Result::OK;
        }

        String commandReply = (moveCommandName == nullptr ? RobotConstants::Result::ERROR_UNKNOWN : *moveCommandName) + " " + status;
        if (status == RobotConstants::Result::OK)
        {
            commandReply += " " + jointPositions();
        }
        if (majMoveTimedOut)
        {
            commandReply += (moveCommandName == nullptr ? RobotConstants::Result::ERROR_UNKNOWN : *moveCommandName) +  " " + RobotConstants::Result::TIMEOUT + " " + jointPositions();
            addDataToOutQueue(commandReply);

            majMoveTimedOut = false;
            majMoveStartMs = 0;
            delay(100);
            //handleMoveHomeJoint(stringToSpeedAcceleration("MHJSP1.0AC0.1", RobotConstants::MoveUnits::UNITS_PERCENT));
            String command_to_home = "MHJ";
            
            RobotConstants::Eeprom::Home data;
            EEPROM.get(RobotConstants::Eeprom::HOME_JOINT_ADDR, data);
            if (data.magic != RobotConstants::Eeprom::HOME_JOINT_MAGIC)
            {
                addDataToOutQueue(RobotConstants::Commands::MOVE_HOME_JOINT + " " + RobotConstants::Result::NO_DATA);
                return;
            }

            String jointsString = jointsToString(data.joints);

            MoveParams<RobotConstants::Robot::AXES_COUNT> moveParams;
            moveParams.status.status = ParamsStatus::OK;
            for (uint8_t i = 0; i < RobotConstants::Robot::AXES_COUNT; ++i)
            {
                moveParams.angles.angles[i] = data.joints[i];
            }
            SpeedAcceleration sa;
            sa.speed = 5;
            sa.acceleration = 1;
            moveParams.move = sa;

            //move(moveParams, true, &command_to_home);
            NVIC_SystemReset();
        }
        addDataToOutQueue(commandReply);

        majMoveTimedOut = false;
        majMoveStartMs = 0;
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
        // else if (axes[nodeId].status == RobotConstants::AxisStatus::NOT_ALIVE && isMAJInProgress)
        // {
        //     DBG_ERROR(DBG_GROUP_MOVE, "MAJ Failed for Axis " + String(nodeId) + ": Axis is not alive (heartbeat timeout)");
        //     axes[nodeId].moveStatus = RobotConstants::MoveStatus::MOVE_FAIL;
        //     MAJ_finalResult();
        // }

        return success;
    }

    bool MoveControllerBase::setMajMoveToleranceSteps(uint8_t nodeId, uint32_t toleranceSteps)
    {
        if (!initialized || nodeId < 1 || nodeId > axesCnt || toleranceSteps < 1)
        {
            return false;
        }
        axes.at(nodeId).setMajMoveToleranceSteps(toleranceSteps);
        return true;
    }

    std::optional<uint32_t> MoveControllerBase::getMajMoveToleranceSteps(uint8_t nodeId) const
    {
        if (!initialized || nodeId < 1 || nodeId > axesCnt)
        {
            return std::nullopt;
        }
        return axes.at(nodeId).getMajMoveToleranceSteps();
    }

    void MoveControllerBase::setMajMoveTimeoutMs(uint32_t ms)
    {
        majMoveTimeoutMs = ms;
    }

    uint32_t MoveControllerBase::getMajMoveTimeoutMs() const
    {
        return majMoveTimeoutMs;
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

    // ======== Request PI Sequence ========
    void MoveControllerBase::RPI_start(uint8_t nodeId)
    {
        // Step 1
        // Step 2
        // Step 3
        canOpen->set_callback_read_PI_controller([this](uint8_t nodeId, uint16_t registerAddress, uint8_t subindex, bool success, int16_t piParameterValue)
                                                 { this->RPI_OnReplyFrom_0x60F9_01(nodeId, success, piParameterValue); });
        // Step 4
        bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::VELOCITY_LOOP_CONTROL, RobotConstants::ODIndices::VELOCITY_KP_SUBINDEX);
        // Step 5
        if (!RPI_checkResponseStatus(nodeId, 0, successSend,
                                     "RPI: Failed to send request 0x60F9:01 for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_read_PI_controller(nullptr);
        }
    }

    void MoveControllerBase::RPI_OnReplyFrom_0x60F9_01(uint8_t nodeId, bool success, int16_t piParameterValue)
    {
        // Step 1
        canOpen->set_callback_read_PI_controller(nullptr);
        // Step 2
        if (!RPI_checkResponseStatus(nodeId, 1, success,
                                     "RPI: Failed to read 0x60F9:01 for Axis " + String(nodeId)))
        {
            return;
        }
        axes.at(nodeId).params.x60F9_velocityControlParameterSet.velocityRegulatorP_gain = piParameterValue;
        // Step 3
        canOpen->set_callback_read_PI_controller([this](uint8_t nodeId, uint16_t registerAddress, uint8_t subindex, bool success, int16_t piParameterValue)
                                                 { this->RPI_OnReplyFrom_0x60F9_02(nodeId, success, piParameterValue); });
        // Step 4
        bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::VELOCITY_LOOP_CONTROL, RobotConstants::ODIndices::VELOCITY_KI_SUBINDEX);
        // Step 5
        if (!RPI_checkResponseStatus(nodeId, 2, successSend,
                                     "RPI: Failed to send request for 0x60F9:02 for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_read_PI_controller(nullptr);
        }
    }

    void MoveControllerBase::RPI_OnReplyFrom_0x60F9_02(uint8_t nodeId, bool success, int16_t piParameterValue)
    {
        // Step 1
        canOpen->set_callback_read_PI_controller(nullptr);
        // Step 2
        if (!RPI_checkResponseStatus(nodeId, 2, success,
                                     "RPI: Failed to read 0x60F9:02 for Axis " + String(nodeId)))
        {
            return;
        }
        axes.at(nodeId).params.x60F9_velocityControlParameterSet.velocityRegulatorI_gain = piParameterValue;
        // Step 3
        canOpen->set_callback_read_PI_controller([this](uint8_t nodeId, uint16_t registerAddress, uint8_t subindex, bool success, int16_t piParameterValue)
                                                 { this->RPI_OnReplyFrom_0x60FB_01(nodeId, success, piParameterValue); });
        // Step 4
        bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::POSITION_LOOP_CONTROL, RobotConstants::ODIndices::POSITION_KP_SUBINDEX);
        // Step 5
        if (!RPI_checkResponseStatus(nodeId, 3, successSend,
                                     "RPI: Failed to send request for 0x60FB:01 for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_read_PI_controller(nullptr);
        }
    }

    void MoveControllerBase::RPI_OnReplyFrom_0x60FB_01(uint8_t nodeId, bool success, int16_t piParameterValue)
    {
        // Step 1
        canOpen->set_callback_read_PI_controller(nullptr);
        // Step 2
        if (!RPI_checkResponseStatus(nodeId, 3, success,
                                     "RPI: Failed to read 0x60FB:01 for Axis " + String(nodeId)))
        {
            return;
        }
        axes.at(nodeId).params.x60FB_positionControlParameterSet.positionRegulatorP_gain = piParameterValue;
        // Step 3
        canOpen->set_callback_read_PI_controller([this](uint8_t nodeId, uint16_t registerAddress, uint8_t subindex, bool success, int16_t piParameterValue)
                                                 { this->RPI_OnReplyFrom_0x60FB_02(nodeId, success, piParameterValue); });
        // Step 4
        bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::POSITION_LOOP_CONTROL, RobotConstants::ODIndices::FEEDFORWARD);
        // Step 5
        if (!RPI_checkResponseStatus(nodeId, 4, successSend,
                                     "RPI: Failed to send request for 0x60FB:02 for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_read_PI_controller(nullptr);
        }
    }

    void MoveControllerBase::RPI_OnReplyFrom_0x60FB_02(uint8_t nodeId, bool success, int16_t piParameterValue)
    {
        // Step 1
        canOpen->set_callback_read_PI_controller(nullptr);
        // Step 2
        if (!RPI_checkResponseStatus(nodeId, 4, success,
                                     "RPI: Failed to read 0x60FB:02 for Axis " + String(nodeId)))
        {
            return;
        }
        axes.at(nodeId).params.x60FB_positionControlParameterSet.velocityFeedForwardFactor = piParameterValue;
        // Need to have some funnel, which will actually print read parameters to Serial
        RPI_finalResult(nodeId, 4);
    }

    void MoveControllerBase::RPI_finalResult(uint8_t nodeId, uint8_t stepsCompleted)
    {
        String reply = RPI_commandName + " ";
        if (stepsCompleted < 4)
        {
            reply += RobotConstants::Result::FAIL + " AN" + String(nodeId);
        }
        else
        {
            reply += RobotConstants::Result::OK + " AN" + String(nodeId) + " VP";
            Axis &a = axes.at(nodeId);
            reply += String(a.params.x60F9_velocityControlParameterSet.velocityRegulatorP_gain) + " VI" +
                     String(a.params.x60F9_velocityControlParameterSet.velocityRegulatorI_gain) + " PP" +
                     String(a.params.x60FB_positionControlParameterSet.positionRegulatorP_gain) + " FF" +
                     String(a.params.x60FB_positionControlParameterSet.velocityFeedForwardFactor);
        }
        addDataToOutQueue(reply);
    }

    bool MoveControllerBase::RPI_checkResponseStatus(uint8_t nodeId, uint8_t step, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "PRI failed for Axis " + String(nodeId) + ": " + errorMessage);
            RPI_finalResult(nodeId, step);
        }
        return success;
    }
    // ======== Request PI Sequence End ========

// ======== Update PI Sequence ========
    void MoveControllerBase::UPS_start(uint8_t nodeId, PIValue piValue)
    {
        UPS_PIValue = piValue;
        // Step 1
        // Step 2
        // Step 3
        canOpen->set_callback_x60F9_01_VP([this](uint8_t nodeId, bool success)
                                    { this->UPS_onReplyFrom_x60F9_01(nodeId, success);}, nodeId);
        // Step 4
        bool successSend = canOpen->x60F9_velocityControlParameterSet_PGain(nodeId, UPS_PIValue.vp);
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send VP " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x60F9_01_VP(nullptr, nodeId);
        }

    }

    void MoveControllerBase::UPS_onReplyFrom_x60F9_01(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x60F9_01_VP(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write VP " + String(nodeId)))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x60F9_02_VI([this](uint8_t nodeId, bool success)
                            { this->UPS_onReplyFrom_x60F9_02(nodeId, success);}, nodeId);
        // Step 4
        bool successSend = canOpen->x60F9_velocityControlParameterSet_IGain(nodeId, UPS_PIValue.vi);        
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send VI " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x60F9_02_VI(nullptr, nodeId);
        }
    }

    void MoveControllerBase::UPS_onReplyFrom_x60F9_02(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x60F9_02_VI(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write VI " + String(nodeId)))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x60FB_01_PP([this](uint8_t nodeId, bool success)
                            { this->UPS_onReplyFrom_x60FB_01(nodeId, success);}, nodeId);
        // Step 4
        bool successSend = canOpen->x60FB_positionControlParameterSet_PGain(nodeId, UPS_PIValue.pp);        
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send PP " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x60FB_01_PP(nullptr, nodeId);
        }
    }

    void MoveControllerBase::UPS_onReplyFrom_x60FB_01(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x60FB_01_PP(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write PP " + String(nodeId)))
        {
            return;
        }

        // Step 3
        canOpen->set_callback_x60FB_02_FF([this](uint8_t nodeId, bool success)
                            { this->UPS_onReplyFrom_x60FB_02(nodeId, success);}, nodeId);
        // Step 4
        bool successSend = canOpen->x60FB_positionControlParameterSet_FeedForwardFactor(nodeId, UPS_PIValue.ff);        
        // Step 5
        if (!ZOE_checkResponseStatus(nodeId, successSend,
                                     "ZOE: Failed to send FF " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x60FB_02_FF(nullptr, nodeId);
        }
    }

    void MoveControllerBase::UPS_onReplyFrom_x60FB_02(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x60FB_02_FF(nullptr, nodeId);
        // Step 2
        if (!ZOE_checkResponseStatus(nodeId, success,
                                     "ZOE: Failed to write FF " + String(nodeId)))
        {
            return;
        }
        // Step 3
        canOpen->set_callback_x2614_dataSaveFlag([this](uint8_t nodeId, bool success)
                                                 { this->UPS_OnReplyFrom_0x2614_DataSaveFlag_Write(nodeId, success); });
        // Step 4
        bool successSend = canOpen->saveParameters(nodeId);
        DBG_INFO(DBG_GROUP_PI, "Sent savePrameters write");        
        // Step 5
        if (!UPS_checkResponseStatus(nodeId, successSend,
                                     "UPS: Failed to send parameter save for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_x2614_dataSaveFlag(nullptr);
        }
    }

    void MoveControllerBase::UPS_OnReplyFrom_0x2614_DataSaveFlag_Write(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->set_callback_x2614_dataSaveFlag(nullptr);
        // Step 2
        if (!UPS_checkResponseStatus(nodeId, success,
                                     "UPS: Failed to write 'save parameters' for Axis " + String(nodeId)))
        {
            return;
        }
        //  Step 3
        canOpen->set_callback_read_x2614_dataSaveFlag([this](uint8_t nodeId, bool success, uint8_t value)
                                                      { this->UPS_OnReplyFrom_0x2614_DataSaveFlag_Read(nodeId, success, value); });
        // Step 4
        delay(2000);
        bool successSend = canOpen->sendSDORead(nodeId,
                                                RobotConstants::ODIndices::DATA_SAVE_FLAG,
                                                RobotConstants::ODIndices::DEFAULT_SUBINDEX);
        DBG_INFO(DBG_GROUP_PI, "Sent saveParameters read");
        // Step 5
        if (!UPS_checkResponseStatus(nodeId, successSend,
                                     "UPS: Failed to send request to read parameter save for Axis " + String(nodeId)))
        {
            // Step 6
            canOpen->set_callback_read_x2614_dataSaveFlag(nullptr);
        }
    }

    void MoveControllerBase::UPS_OnReplyFrom_0x2614_DataSaveFlag_Read(uint8_t nodeId, bool success, uint8_t value)
    {
        // Step 1
        canOpen->set_callback_read_x2614_dataSaveFlag(nullptr);
        // Step 2
        if (!UPS_checkResponseStatus(nodeId, success,
                                     "UPS: Failed to read 'save parameters' for Axis " + String(nodeId)))
        {
            return;
        }
        String reply = RobotConstants::Commands::PI_CONTROLLER_WRITE + " ";
        if (value == 2)
        {
            startRequestPI(nodeId, RobotConstants::Commands::PI_CONTROLLER_WRITE);
            return;
        }
        addDataToOutQueue(RobotConstants::Result::FAIL + " AN" + String(nodeId));
    }

    bool MoveControllerBase::UPS_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
            DBG_ERROR(DBG_GROUP_MOVE, "UPS Failed for Axis " + String(nodeId) + ": " + errorMessage);
            addDataToOutQueue(RobotConstants::Commands::PI_CONTROLLER_WRITE + " " +
                              RobotConstants::Result::FAIL + " " +
                              " AN" + String(nodeId));
            UPS_PIValue.nodeId = 0;
        }
        return success;
    }

    // ======== Update PI Sequence End ========

    // ======== End of Fixate Axis after restoring life ========

    String MoveControllerBase::prepareMoveStatusToString(PrepareMoveStatus status)
    {
        switch (status)
        {
        case PrepareMoveStatus::OK:
            return "OK";
        case PrepareMoveStatus::OUT_OF_LIMITS:
            return "IL";
        case PrepareMoveStatus::FAIL:
            return "FF";
        case PrepareMoveStatus::OTHER_COMMAND_IN_PROGRESS:
            return "MP";
        case PrepareMoveStatus::INVALID_PROFILE:
            return "PF";
        case PrepareMoveStatus::INVALID_PARAMS:
            return "IP";
        default:
            return RobotConstants::Result::ERROR_UNKNOWN;
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

    // void MoveControllerBase::setPIControlParameter(uint8_t nodeId, uint8_t parameterId, int16_t value)
    // {
    //     UPS_start(nodeId, parameterId, value);
    // }

    String MoveControllerBase::jointPositions(bool inSteps)
    {
        if (!initialized)
        {
            return "";
        }
        String positions = "";
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            positions += String((char)RobotConstants::Robot::AXIS_IDENTIFIER_CHAR) + String((char)(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1));
            if (inSteps)
            {
                positions += String(axisPosition(nodeId).value_or(0)) + " ";
            }
            else
            {
                positions += String(Axis::stepsToDegrees(axisPosition(nodeId).value_or(0)), 3) + " ";
            }
        }
        return positions;
    }

    String MoveControllerBase::cartesianPosition()
    {
        if (!initialized || !solver)
        {
            return "";
        }
        JointAngles<RobotConstants::Robot::AXES_COUNT> jointAngles;
        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            jointAngles.angles[nodeId - 1] = Axis::stepsToRadians(axisPosition(nodeId).value_or(0));
        }

        return solver->fk(jointAngles).toStr();
    }

    void MoveControllerBase::setPIController(uint8_t nodeId, PIValue piValue)
    {
        UPS_start(piValue.nodeId, piValue);
    }

}