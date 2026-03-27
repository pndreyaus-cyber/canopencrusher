#include <math.h>
#include <iostream>

#include "Arduino.h"
#include "Axis.h"
#include "RobotConstants.h"
#include "Debug.h"

extern void addDataToOutQueue(String data);

namespace StepDirController
{

    Axis::Axis() : nodeId(kInvalidNodeId)
    {
        initialized = false;
        initStatus = RobotConstants::InitStatus::ZEI_NONE;
        moveStatus = RobotConstants::MoveStatus::NOT_TASKED_WITH_MOVE;
        status = RobotConstants::AxisStatus::NOT_ALIVE;
    }

    Axis::Axis(uint8_t nodeId, bool reversedLogic) : nodeId(nodeId), reversedLogic(reversedLogic)
    {
        init_od_ram(&params);
        params.x6064_positionActualValue = 0;
        initStatus = RobotConstants::InitStatus::ZEI_NONE;
        moveStatus = RobotConstants::MoveStatus::NOT_TASKED_WITH_MOVE;
        status = RobotConstants::AxisStatus::NOT_ALIVE;
        initialized = true;
        limitsEnabled = false;
    }

    // ===================== Setters =====================
    bool Axis::reverseLogic()
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::reverseLogic -- Axis not initialized");
            return false;
        }

        reversedLogic = !reversedLogic;

        if (limitsEnabled)
        {
            int32_t tempLowLimitSteps = lowLimitSteps;
            lowLimitSteps = -highLimitSteps;
            highLimitSteps = -tempLowLimitSteps;
        }

        return true;
    }

    bool Axis::checkTargetPositionInStepsForLimits(int32_t targetPositionInSteps)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::checkTargetPositionInStepsForLimits -- Axis not initialized");
            return false;
        }

        return (lowLimitSteps - RobotConstants::Axis::LIMIT_TOLERANCE) <= targetPositionInSteps && targetPositionInSteps <= (highLimitSteps + RobotConstants::Axis::LIMIT_TOLERANCE);
    }

    bool Axis::setTargetPositionInUnits(double units)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setTargetPositionInUnits -- Axis not initialized");
            return false;
        }

        return setTargetPositionInSteps(unitsToSteps(units));
    }

    bool Axis::setTargetPositionInSteps(int32_t steps)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setTargetPositionInSteps -- Axis not initialized");
            return false;
        }
        if (limitsEnabled && !checkTargetPositionInStepsForLimits(steps))
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setTargetPositionInSteps -- Target position out of limits");
            return false;
        }

        if (reversedLogic)
        {
            steps = -steps;
        }
        params.x607A_targetPosition = steps;

        return true;
    }

    bool Axis::setProfileVelocityInUnitsPerSec(double velocityUnits)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setProfileVelocityInUnitsPerSec -- Axis not initialized");
            return false;
        }

        return setProfileVelocityInRPM(speedUnitsToMotorRPM(velocityUnits));
    }

    bool Axis::setProfileVelocityInRPM(uint32_t rpm)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setProfileVelocityInRPM -- Axis not initialized");
            return false;
        }

        params.x6081_profileVelocity = rpm;
        return true;
    }

    bool Axis::setProfileAccelerationInUnitsPerSec2(double accelerationUnits)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setProfileAccelerationInUnitsPerSec2 -- Axis not initialized");
            return false;
        }

        return setProfileAccelerationInRPMPerSec(accelerationUnitsToRPMPS(accelerationUnits));
    }

    bool Axis::setProfileAccelerationInRPMPerSec(uint32_t rpmPerSec)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setProfileAccelerationInRPMPerSec -- Axis not initialized");
            return false;
        }

        params.x6083_profileAcceleration = rpmPerSec;
        return true;
    }
    // ===================== Setters end =====================

    // ===================== Getters =====================
    std::optional<uint8_t> Axis::getNodeId() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getNodeId -- Axis not initialized");
            return std::nullopt;
        }

        return nodeId;
    }

    std::optional<int32_t> Axis::getCurrentPositionInSteps() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getCurrentPositionInSteps -- Axis not initialized");
            return std::nullopt;
        }
        int32_t positionActualValue = params.x6064_positionActualValue;
        if (reversedLogic)
        {
            return -positionActualValue;
        }
        else
        {
            return positionActualValue;
        }
    }

    std::optional<int32_t> Axis::getTargetPositionInSteps() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getTargetPositionInSteps -- Axis not initialized");
            return std::nullopt;
        }
        int32_t targetPosition = params.x607A_targetPosition;
        // if (reversedLogic)
        // {
        //     return -targetPosition;
        // }
        // else
        // {
        //     return targetPosition;
        // }
        return targetPosition;
    }

    std::optional<uint32_t> Axis::getProfileVelocityInRPM() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getProfileVelocityInRPM -- Axis not initialized");
            return std::nullopt;
        }

        return params.x6081_profileVelocity;
    }

    std::optional<uint32_t> Axis::getProfileAccelerationInRPMPerSec() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getProfileAccelerationInRPMPerSec -- Axis not initialized");
            return std::nullopt;
        }

        return params.x6083_profileAcceleration;
    }

    // ===================== Getters end =====================

    // ============================= Static methods =============================
    double Axis::stepsToUnits(int32_t steps) // Convert steps to degrees
    {
        return steps * RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV / (RobotConstants::Axis::GEAR_RATIO * RobotConstants::Axis::STEPS_PER_MOTOR_REV);
    }

    int32_t Axis::unitsToSteps(double units) // Convert degrees to steps
    {
        return static_cast<int32_t>(units * RobotConstants::Axis::GEAR_RATIO * RobotConstants::Axis::STEPS_PER_MOTOR_REV / RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV);
    }

    uint32_t Axis::speedUnitsToMotorRPM(double speedUnits) // Convert degrees/sec to RPM
    {
        if (speedUnits < 0)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::speedUnitsToMotorRPM -- negative speed units not allowed");
            return 0;
        }

        return static_cast<uint32_t>(speedUnits * RobotConstants::Math::SECONDS_IN_MINUTE * RobotConstants::Axis::GEAR_RATIO / RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV);
    }

    double Axis::motorRPMToSpeedUnits(uint32_t rpm) // Convert RPM to degrees/sec
    {
        return static_cast<double>(rpm) * RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV / (RobotConstants::Math::SECONDS_IN_MINUTE * RobotConstants::Axis::GEAR_RATIO);
    }

    double Axis::motorRPMToStepsPerSec(uint32_t rpm)
    {
        return static_cast<double>(rpm) * RobotConstants::Axis::STEPS_PER_MOTOR_REV / RobotConstants::Math::SECONDS_IN_MINUTE;
    }

    double Axis::motorRPMPSToStepsPerSec2(uint32_t rpmPerSec)
    {
        return static_cast<double>(rpmPerSec) * RobotConstants::Axis::STEPS_PER_MOTOR_REV / RobotConstants::Math::SECONDS_IN_MINUTE;
    }

    uint32_t Axis::stepsPerSecToMotorRPM(double stepsPerSec) // Convert steps/sec to RPM
    {
        return static_cast<uint32_t>(std::ceil(Axis::stepsPerSecToMotorRPMDouble(stepsPerSec)));
    }

    uint32_t Axis::stepsPerSec2ToRPMPS(double stepsPerSec2) // Convert degrees/sec^2 to RPM/sec
    {
        return static_cast<uint32_t>(std::ceil(Axis::stepsPerSec2ToRPMPSDouble(stepsPerSec2)));
    }

    double Axis::stepsPerSecToMotorRPMDouble(double stepsPerSec) // Convert steps/sec to RPM
    {
        if (stepsPerSec < 0)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::stepsPerSecToMotorRPMDouble -- negative steps/sec not allowed");
            return 0;
        }
        return stepsPerSec * RobotConstants::Math::SECONDS_IN_MINUTE / RobotConstants::Axis::STEPS_PER_MOTOR_REV;
    }

    double Axis::stepsPerSec2ToRPMPSDouble(double stepsPerSec2) // Convert degrees/sec^2 to RPM/sec
    {
        if (stepsPerSec2 < 0)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::stepsPerSec2ToRPMPSDouble -- negative steps/sec^2 not allowed");
            return 0;
        }
        return stepsPerSec2 * RobotConstants::Math::SECONDS_IN_MINUTE / RobotConstants::Axis::STEPS_PER_MOTOR_REV;
    }

    uint32_t Axis::accelerationUnitsToRPMPS(double accelerationUnits) // Convert degrees/sec^2 to rev/(min*sec)
    {
        if (accelerationUnits < 0)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::accelerationUnitsToRPMPS -- negative acceleration units not allowed");
            return 0;
        }
        return static_cast<uint32_t>(accelerationUnits * RobotConstants::Math::SECONDS_IN_MINUTE * RobotConstants::Axis::GEAR_RATIO / RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV);
    }

    double Axis::RPMPSToAccelerationUnits(uint32_t rpmPerSecond)
    {
        return static_cast<double>(rpmPerSecond) * RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV / (RobotConstants::Math::SECONDS_IN_MINUTE * RobotConstants::Axis::GEAR_RATIO);
    }

    double Axis::stepsToMotorRevs(int32_t steps)
    {
        return static_cast<double>(steps) / RobotConstants::Axis::STEPS_PER_MOTOR_REV;
    }
    // ============================= Static methods end =============================

    // ============================= Protected methods =============================
    bool Axis::setCurrentPositionInSteps(int32_t steps)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setCurrentPositionInSteps -- Axis not initialized");
            return false;
        }

        params.x6064_positionActualValue = steps;

        return true;
    }

    bool Axis::setLimits(double lowLimitUnits, double highLimitUnits)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setLimits -- Axis not initialized");
            return false;
        }
        if (lowLimitUnits > highLimitUnits)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setLimits -- low limit must be less than high limit");
            return false;
        }

        lowLimitSteps = unitsToSteps(lowLimitUnits);
        highLimitSteps = unitsToSteps(highLimitUnits);
        if(reversedLogic)
        {
            lowLimitSteps = -lowLimitSteps;
            highLimitSteps = -highLimitSteps;
        }

        return true;
    }
    // ============================= Protected methods end =============================
}