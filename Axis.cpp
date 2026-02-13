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
        status = RobotConstants::AxisStatus::OPERATIONAL;
    }

    Axis::Axis(uint8_t nodeId) : nodeId(nodeId)
    {
        init_od_ram(&params);
        params.x6064_positionActualValue = 0;
        initStatus = RobotConstants::InitStatus::ZEI_NONE;
        status = RobotConstants::AxisStatus::OPERATIONAL;
        initialized = true;
    }

    // ===================== Setters =====================
    void Axis::setCurrentPositionInSteps(int32_t steps)
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::setCurrentPositionInSteps -- Axis not initialized");
            return;
        }

        params.x6064_positionActualValue = steps;
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

        params.x607A_targetPosition = steps;
        relativeMovementSteps = steps - getCurrentPositionInSteps();

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
    uint8_t Axis::getNodeId() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getNodeId -- Axis not initialized");
            return 0;
        }

        return nodeId;
    }

    int32_t Axis::getCurrentPositionInSteps() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getCurrentPositionInSteps -- Axis not initialized");
            return -1;
        }

        return params.x6064_positionActualValue;
    }

    int32_t Axis::getRelativeMovementInSteps() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getRelativeMovementInSteps -- Axis not initialized");
            return -1;
        }

        return relativeMovementSteps;
    }

    int32_t Axis::getTargetPositionInSteps() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getTargetPositionInSteps -- Axis not initialized");
            return -1;
        }

        return params.x607A_targetPosition;
    }

    uint32_t Axis::getProfileVelocityInRPM() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getProfileVelocityInRPM -- Axis not initialized");
            return -1;
        }

        return params.x6081_profileVelocity;
    }

    uint32_t Axis::getProfileAccelerationInRPMPerSec() const
    {
        if (!initialized)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::getProfileAccelerationInRPMPerSec -- Axis not initialized");
            return -1;
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
    
    uint32_t Axis::stepsPerSecToMotorRPM(double stepsPerSec) // Convert steps/sec to RPM
    {
        if (stepsPerSec < 0)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::stepsPerSecToMotorRPM -- negative steps/sec not allowed");
            return 0;
        }

        return static_cast<uint32_t>(stepsPerSec * RobotConstants::Math::SECONDS_IN_MINUTE / RobotConstants::Axis::STEPS_PER_MOTOR_REV);
    }

    uint32_t Axis::accelerationUnitsToRPMPS(double accelearionUnits) // Перевести градусы/сек^2 в об/(мин*сек)
    {
        if (accelearionUnits < 0)
        {
            DBG_WARN(DBG_GROUP_AXIS, "Axis::accelerationUnitsToRPMPS -- negative acceleration units not allowed");
            return 0;
        }
        return static_cast<uint32_t>(accelearionUnits * RobotConstants::Math::SECONDS_IN_MINUTE * RobotConstants::Axis::GEAR_RATIO / RobotConstants::Axis::UNITS_PER_OUTPUT_SHAFT_REV);
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
}