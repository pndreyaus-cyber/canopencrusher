#include "Axis.h"
#include <math.h>
#include <iostream>
#include "Arduino.h"


namespace StepDirController{
Axis::Axis() : nodeId(0)
{
}

Axis::Axis(const uint8_t nodeId) : nodeId(nodeId), canOpenCharacteristics()
{
    init_od_ram(&canOpenCharacteristics);
}

Axis &Axis::setStepsPerRevolution(uint32_t steps)
{
    stepsPerRevolution = steps;
    return *this;
}

Axis &Axis::setUnitsPerRevolution(double units)
{
    unitsPerRevolution = units;
    return *this;
}

Axis &Axis::enableLimits(double minUnits, double maxUnits)
{
    minPositionUnits = maxUnits > minUnits ? minUnits : maxUnits;
    maxPositionUnits = maxUnits > minUnits ? maxUnits : minUnits;
    return enableLimits();
}

Axis &Axis::enableLimits()
{
    usePositionLimits = true;
    return *this;
}

Axis &Axis::disableLimits()
{
    usePositionLimits = false;
    return *this;
}

Axis &Axis::setCurrentPositionInUnits(double units)
{
    int32_t steps = std::round(unitsToSteps(units)); // Edited for C++
    return setCurrentPositionInSteps(steps);
}

Axis &Axis::setCurrentPositionInSteps(int32_t steps)
{
    currentPosition = steps;
    canOpenCharacteristics.x6064_positionActualValue = steps;
    return *this;
}

int32_t Axis::getCurrentPositionInSteps() const
{
    return canOpenCharacteristics.x6064_positionActualValue;
}

bool Axis::setTargetPositionRelativeInUnits(double units)
{
    int32_t steps = std::round(unitsToSteps(units)); // Edited for c++

    return setTargetPositionRelativeInSteps(steps);
}

bool Axis::setTargetPositionRelativeInSteps(int32_t steps)
{
    return setTargetPositionAbsoluteInSteps(steps + canOpenCharacteristics.x6064_positionActualValue); // Проверить
}

bool Axis::setTargetPositionAbsoluteInUnits(double units)
{

    int32_t steps = std::round(unitsToSteps(units)); // Edited for c++
    return setTargetPositionAbsoluteInSteps(steps);      

}

bool Axis::setTargetPositionAbsoluteInSteps(int32_t steps)
{
    int32_t relativePosition = steps - canOpenCharacteristics.x6064_positionActualValue;
    movementUnits = stepsToUnits(relativePosition);
    movementSteps = std::fabs(relativePosition);
    canOpenCharacteristics.x607A_targetPosition = steps;
    return true;
}

double Axis::getPositionInUnits() const
{
    double position = stepsToUnits(canOpenCharacteristics.x6064_positionActualValue);
    return position;
}

int32_t Axis::getPositionInSteps() const
{
    int32_t position = canOpenCharacteristics.x6064_positionActualValue;
    return position;
}

uint32_t Axis::getStepsPerRevolution() const
{
    return stepsPerRevolution;
}

double Axis::getUnitsPerRevolution() const
{
    return unitsPerRevolution;
}

double Axis::stepsToUnits(int32_t steps) const // Перевести шаги в градусы
{
    if(stepsPerRevolution == 0 || unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::stepsToUnits -- division by zero"); 
        return 0;
    }
    return steps / (double)stepsPerRevolution * unitsPerRevolution;
}

int32_t Axis::unitsToSteps(double units) const // Перевести градусы в шаги
{
    if(stepsPerRevolution == 0 || unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::unitsToSteps -- division by zero");
        return 0;
    }
    return units / unitsPerRevolution * stepsPerRevolution;
}

uint32_t Axis::speedUnitsToRevolutionsPerMinute(double speedUnits) const // Перевести градусы/сек в об/мин
{
    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::speedUnitsToRevolutionsPerMinute -- division by zero");
        return 0;
    }
    return speedUnits * SECONDS_IN_MINUTE / unitsPerRevolution;
}

double Axis::revolutionsPerMinuteToSpeedUnits(uint32_t rpm) const // Перевести из об/мин в градусы/сек
{
    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::revolutionsPerMinuteToSpeedUnits -- division by zero");
        return 0;
    }
    return (double)rpm / SECONDS_IN_MINUTE * unitsPerRevolution;
}

uint32_t Axis::accelerationUnitsTorpmPerSecond(double accelearionUnits) const // Перевести градусы/сек^2 в об/(мин*сек)
{
    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::accelerationUnitsTorpmPerSecond -- division by zero");
        return 0;
    }
    return accelearionUnits * SECONDS_IN_MINUTE / unitsPerRevolution;
}

double Axis::rpmPerSecondToAccelerationUnits(double rpmPerSecond) const  // Перевести об/(мин*сек) в градусы/сек^2
{
    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::rpmPerSecondToAccelerationUnits -- division by zero");
        return 0;
    }   
    return (double)rpmPerSecond / SECONDS_IN_MINUTE * unitsPerRevolution; // TODO: потенциальная потеря точности расчетов из-за SECONDS_IN_MINUTE, лучше объявить как константу float/double
}

double Axis::getMaxLimitUnits() const
{
    return maxPositionUnits;
}

double Axis::getMinLimitUnits() const
{
    return minPositionUnits;
}

uint8_t Axis::getNodeId() const
{
    return nodeId;
}

double Axis::getMovementUnits() const
{
    return movementUnits;
}

int32_t Axis::getTargetPositionAbsolute() const
{
    return canOpenCharacteristics.x607A_targetPosition;
}

}