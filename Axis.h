#pragma once
#ifndef AXIS_H

#define AXIS_H

#include <cstdint>
#include <optional>

#include "OD.h"
#include "objdict_objectdefines.h"
#include "RobotConstants.h"

namespace StepDirController
{
    constexpr uint8_t kInvalidNodeId = 0xFF; // Sentinel value indicating an uninitialized or invalid axis node ID"

    class Axis
    {
    public:
        Axis();
        Axis(uint8_t nodeId, bool reversedLogic = false);

        // ============================= Setters of target position, profile velocity and acceleration =============================
        bool setCurrentPositionInSteps(int32_t steps);

        bool setTargetPositionInUnits(double units);
        bool setTargetPositionInSteps(int32_t steps);

        bool setProfileVelocityInUnitsPerSec(double velocityUnits);
        bool setProfileVelocityInRPM(uint32_t rpm);

        bool setProfileAccelerationInUnitsPerSec2(double accelerationUnits);
        bool setProfileAccelerationInRPMPerSec(uint32_t rpmPerSec);
        // ============================= Setters of target position, profile velocity and acceleration end =============================

        // ============================= Getters =============================
        std::optional<uint8_t> getNodeId() const;
        std::optional<int32_t> getCurrentPositionInSteps() const;
        std::optional<int32_t> getTargetPositionInSteps() const;
        std::optional<uint32_t> getProfileVelocityInRPM() const;
        std::optional<uint32_t> getProfileAccelerationInRPMPerSec() const;
        // ============================= Getters end =============================

        // ============================= Static methods =============================
        static double stepsToUnits(int32_t steps); // Convert steps to units (degrees)
        static int32_t unitsToSteps(double units); // Convert units (degrees) to steps

        static uint32_t speedUnitsToMotorRPM(double speedUnits); // Convert degrees/sec to RPM
        static double motorRPMToSpeedUnits(uint32_t rpm);        // Convert RPM to degrees/sec

        static double motorRPMToStepsPerSec(uint32_t rpm);          // Convert RPM to steps/sec
        static double motorRPMPSToStepsPerSec2(uint32_t rpmPerSec); // Convert RPM/sec to steps/sec^2

        static uint32_t stepsPerSecToMotorRPM(double stepsPerSec); // Convert steps/sec to RPM
        static uint32_t stepsPerSec2ToRPMPS(double stepsPerSec2);  // Convert degrees/sec^2 to RPM/sec

        static double stepsPerSecToMotorRPMDouble(double stepsPerSec); // Convert steps/sec to RPM
        static double stepsPerSec2ToRPMPSDouble(double stepsPerSec2);  // Convert degrees/sec^2 to RPM/sec

        static uint32_t accelerationUnitsToRPMPS(double accelerationUnits); // Convert degrees/sec^2 to RPM/sec
        static double RPMPSToAccelerationUnits(uint32_t rpmPerSecond);      // Convert RPM/sec to degrees/sec^2

        static double stepsToMotorRevs(int32_t steps);
        // ============================= Static methods end =============================

    protected:
        bool initialized = false;
        uint8_t nodeId;
        OD_RAM_t params;

        bool reversedLogic = false; // Whether the direction of movement is reversed for this axis

        double regularSpeed; // крейсерская скорость в шагах/сек
        double acceleration; // ускорение в шагах/сек^2

        friend class MoveControllerBase;

        // For ZEI
        RobotConstants::InitStatus initStatus;
        uint32_t lastHeartbeatMs = 0;
        // bool isAlive = true;

        // For MAJ
        uint32_t lastRequestedStatusWord = 0;
        RobotConstants::MoveStatus moveStatus;

        // Status
        RobotConstants::AxisStatus status;

    };
}

#endif