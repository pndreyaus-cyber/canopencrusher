#pragma once
#ifndef AXIS_H
#define AXIS_H

#include <cstdint>
//#include <Arduino.h>
#include "OD.h"
#include "objdict_objectdefines.h"

//#define STEPS_PER_REVOLUTION 50 * 32768
//#define UNITS_PER_REVOLUTION 360
#define SECONDS_IN_MINUTE 60

/*
enum class Direction {
    POSITIVE,
    NEGATIVE
};
*/
namespace StepDirController{
class Axis{
    public:
        Axis();
        Axis(const uint8_t nodeId);
    

        Axis &setStepsPerRevolution(uint32_t steps); // +
        Axis &setUnitsPerRevolution(double units); // +

        Axis &enableLimits(double minUnits, double maxUnits); // + включение ограничений на минимальное и максимальное положение оси в единицах измерения.
        Axis &enableLimits(); // + включение ограничений на минимальное и максимальное положение оси в единицах измерения.
        Axis &disableLimits(); // + отключение ограничений на положение оси

        // Axis &enable();
        // Axis &disable();

        //Axis &setInverseDirPin(bool isInverted); // +
        //Axis &setInverseStepPin(bool isInverted); // +
        //Axis &setInverseEnablePin(bool isInverted); // +

        Axis &setCurrentPositionInUnits(double units); // +
        Axis &setCurrentPositionInSteps(int32_t steps); // +

       // Axis &setOnStepDone(void (*callback)()); // +

        bool setTargetPositionAbsoluteInUnits(double units);
        bool setTargetPositionAbsoluteInSteps(int32_t steps);
        bool setTargetPositionRelativeInUnits(double units);
        bool setTargetPositionRelativeInSteps(int32_t steps);

        double getMovementUnits() const;
        int32_t getTargetPositionAbsolute() const;

        double getPositionInUnits() const;
        int32_t getPositionInSteps() const;

        uint32_t getStepsPerRevolution() const;
        double getUnitsPerRevolution() const;

        double stepsToUnits(int32_t steps) const; // Перевести шаги в градусы
        int32_t unitsToSteps(double units) const; // Перевести градусы в шаги

        double getMinLimitUnits() const;
        double getMaxLimitUnits() const;

        uint32_t speedUnitsToRevolutionsPerMinute(double speedUnits) const; // Перевести градусы/сек в об/мин
        double revolutionsPerMinuteToSpeedUnits(uint32_t rpm) const; // Перевести из об/мин в градусы/сек

        uint32_t accelerationUnitsTorpmPerSecond(double accelearionUnits) const; // Перевести градусы/сек^2 в об/(мин*сек)
        double rpmPerSecondToAccelerationUnits(double rpmPerSecond) const;  // Перевести об/(мин*сек) в градусы/сек^2

        uint8_t getNodeId() const;

        int32_t getCurrentPositionInSteps() const;

    protected:
        const uint8_t nodeId;
        OD_RAM_t canOpenCharacteristics;

        uint32_t stepsPerRevolution = 0; // количество шагов на оборот
        double unitsPerRevolution = 0; // количество единиц измерения на оборот

        double maxPositionUnits = 0.0f; // максимальное положение оси в единицах измерения
        double minPositionUnits = 0.0f; // минимальное положение оси в единицах измерения

        bool usePositionLimits = false; // флаг использования максимального и минимального положения оси

        // bool inverseDir = false;
        // bool inverseStep = false;
        // bool inverseEnable = false;

        //Direction direction = Direction::POSITIVE;

        double movementUnits; // относительное перемещение в единицах измерения; используется для расчета синхронизации осей
        volatile uint32_t movementSteps; // относительное перемещение в шагах; 
        //volatile uint32_t movementStepsInWork; // относительное перемещение в шагах в работе; 

        volatile int32_t currentPosition; // текущее положение в шагах 
        
        //volatile uint32_t startStepTime; // время начала текущего шага (в микросекундах)
        //volatile uint32_t decelerationStartTime; // время начала замедления (в микросекундах)
        //volatile uint32_t stepDuration; // длительность шага (в микросекундах)

        //volatile uint32_t stepsDone; // выполнено шагов в текущем задании

        //double startSpeed; // начальная скорость (рывок) в шагах/сек
        double regularSpeed; // крейсерская скорость в шагах/сек
        //double endSpeed; // конечная скорость в шагах/сек
        double acceleration; // ускорение в шагах/сек^2
        //double decleleration; // замедление в шагах/сек^2

        //uint32_t accelerationSteps; // количество шагов на разгон
        //volatile uint32_t decelerationStartStep; // номер шага с которого начнется замедление

        //Axis &setDirection(Direction newDirection);
        //Axis &stepOn();
        //Axis &stepOff();
        //Axis &stepDone();

        void (*onStepDone)() = nullptr; // callback на каждом шаге

        friend class MoveControllerBase;
        // friend class ControllerBase;
};
}

#endif