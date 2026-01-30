#pragma once
#ifndef AXIS_H

#define AXIS_H

#include <cstdint>
#include "OD.h"
#include "objdict_objectdefines.h"
#include "RobotConstants.h"

namespace StepDirController{
constexpr uint8_t kInvalidNodeId = 0xFF; //Sentinel value indicating an uninitialized or invalid axis node ID"

class Axis{
    public:
        Axis();
        Axis(uint8_t nodeId);

        Axis &setStepsPerRevolution(uint32_t steps); // +
        Axis &setUnitsPerRevolution(double units); // +

        Axis &setCurrentPositionInUnits(double units); // +
        Axis &setCurrentPositionInSteps(int32_t steps); // +

        bool setTargetPositionAbsoluteInUnits(double units);
        bool setTargetPositionAbsoluteInSteps(int32_t steps);
        bool setTargetPositionRelativeInUnits(double units);
        bool setTargetPositionRelativeInSteps(int32_t steps);

        double getMovementUnits() const;
        int32_t getTargetPositionAbsolute() const;

        double getPositionInUnits() const;

        uint32_t getStepsPerRevolution() const;
        double getUnitsPerRevolution() const;

        double stepsToUnits(int32_t steps) const; // Перевести шаги в градусы
        int32_t unitsToSteps(double units) const; // Перевести градусы в шаги

        uint32_t speedUnitsToRevolutionsPerMinute(double speedUnits) const; // Перевести градусы/сек в об/мин
        double revolutionsPerMinuteToSpeedUnits(uint32_t rpm) const; // Перевести из об/мин в градусы/сек

        uint32_t accelerationUnitsTorpmPerSecond(double accelearionUnits) const; // Перевести градусы/сек^2 в об/(мин*сек)
        double rpmPerSecondToAccelerationUnits(double rpmPerSecond) const;  // Перевести об/(мин*сек) в градусы/сек^2

        uint8_t getNodeId() const;

        int32_t getCurrentPositionInSteps() const;

    protected:
        bool initialized = false;
        uint8_t nodeId;
        OD_RAM_t params;

        uint32_t stepsPerRevolution = 0; // количество шагов на оборот
        double unitsPerRevolution = 0; // количество единиц измерения на оборот

        double movementUnits; // относительное перемещение в единицах измерения; используется для расчета синхронизации осей
        volatile uint32_t movementSteps; // относительное перемещение в шагах; 

        double regularSpeed; // крейсерская скорость в шагах/сек
        double acceleration; // ускорение в шагах/сек^2

        friend class MoveControllerBase;
};
}

#endif