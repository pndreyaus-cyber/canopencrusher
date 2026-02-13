#pragma once
#ifndef AXIS_H

#define AXIS_H

#include <cstdint>
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
        Axis(uint8_t nodeId);

        // ============================= Setters of target position, profile velocity and acceleration =============================
        bool setTargetPositionInUnits(double units);
        bool setTargetPositionInSteps(int32_t steps);

        bool setProfileVelocityInUnitsPerSec(double velocityUnits);
        bool setProfileVelocityInRPM(uint32_t rpm);

        bool setProfileAccelerationInUnitsPerSec2(double accelerationUnits);
        bool setProfileAccelerationInRPMPerSec(uint32_t rpmPerSec);
        // ============================= Setters of target position, profile velocity and acceleration end =============================

        // ============================= Getters =============================
        uint8_t getNodeId() const;
        int32_t getCurrentPositionInSteps() const;
        int32_t getRelativeMovementInSteps() const;
        int32_t getTargetPositionInSteps() const;
        uint32_t getProfileVelocityInRPM() const;
        uint32_t getProfileAccelerationInRPMPerSec() const;
        // ============================= Getters end =============================

        // ============================= Static methods =============================
        static double stepsToUnits(int32_t steps); // Convert steps to units (degrees)
        static int32_t unitsToSteps(double units); // Convert units (degrees) to steps

        static uint32_t speedUnitsToMotorRPM(double speedUnits); // Convert degrees/sec to RPM
        static double motorRPMToSpeedUnits(uint32_t rpm); // Convert RPM to degrees/sec

        static uint32_t stepsPerSecToMotorRPM(double stepsPerSec); // Convert steps/sec to RPM

        static uint32_t accelerationUnitsToRPMPS(double accelearionUnits); // Convert degrees/sec^2 to RPM/sec
        static double RPMPSToAccelerationUnits(uint32_t rpmPerSecond);     // Convert RPM/sec to degrees/sec^2

        static double stepsToMotorRevs(int32_t steps);
        // ============================= Static methods end =============================

    protected:
        bool initialized = false;
        uint8_t nodeId;
        OD_RAM_t params;

        int32_t relativeMovementSteps;

        double regularSpeed; // крейсерская скорость в шагах/сек
        double acceleration; // ускорение в шагах/сек^2

        friend class MoveControllerBase;

        // For ZEI
        RobotConstants::InitStatus initStatus;
        RobotConstants::AxisStatus status;
        uint32_t lastHeartbeatMs = 0;
        bool isAlive = true;

        // For MAJ
        uint32_t lastRequestedStatusWord = 0;;

        void setCurrentPositionInSteps(int32_t steps);
    };
}

#endif