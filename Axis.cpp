#include <math.h>
#include <iostream>
#include "Arduino.h"
#include "Axis.h"
#include "RobotConstants.h"

namespace StepDirController{

Axis::Axis() : nodeId(kInvalidNodeId) {}

Axis::Axis(uint8_t nodeId) : nodeId(nodeId) {
    init_od_ram(&canOpenCharacteristics);
    currentPosition = 0;
    movementUnits = 0.0;
    canOpenCharacteristics.x6064_positionActualValue = 0;
    stepsPerRevolution = RobotConstants::Axis::DEFAULT_STEPS_PER_REVOLUTION;
    unitsPerRevolution = RobotConstants::Axis::DEFAULT_UNITS_PER_REVOLUTION;
    initialized = true;
}

Axis &Axis::setStepsPerRevolution(uint32_t steps)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setStepsPerRevolution -- Axis not initialized");
#endif        
        return *this;
    }
    stepsPerRevolution = steps;
    return *this;
}

Axis &Axis::setUnitsPerRevolution(double units)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setUnitsPerRevolution -- Axis not initialized");
#endif        
        return *this;
    }
    unitsPerRevolution = units;
    return *this;
}

Axis &Axis::enableLimits(double minUnits, double maxUnits)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::enableLimits -- Axis not initialized");
#endif        
        return *this;
    }

    minPositionUnits = maxUnits > minUnits ? minUnits : maxUnits;
    maxPositionUnits = maxUnits > minUnits ? maxUnits : minUnits;
    return enableLimits();
}

Axis &Axis::enableLimits()
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::enableLimits -- Axis not initialized");
#endif        
        return *this;
    }

    usePositionLimits = true;
    return *this;
}

Axis &Axis::disableLimits()
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::disableLimits -- Axis not initialized");
#endif        
        return *this;
    }
      
    usePositionLimits = false;
    return *this;
}

Axis &Axis::setCurrentPositionInUnits(double units)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setCurrentPositionInUnits -- Axis not initialized");
#endif        
        return *this;
    }
      
    int32_t steps = std::round(unitsToSteps(units)); // Edited for C++
    return setCurrentPositionInSteps(steps);
}

Axis &Axis::setCurrentPositionInSteps(int32_t steps)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setCurrentPositionInSteps -- Axis not initialized");
#endif        
        return *this;
    }
      
    currentPosition = steps;
    canOpenCharacteristics.x6064_positionActualValue = steps;
    return *this;
}

int32_t Axis::getCurrentPositionInSteps() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getCurrentPositionInSteps -- Axis not initialized");
#endif        
        return -1;
    }
      
    return canOpenCharacteristics.x6064_positionActualValue;
}

bool Axis::setTargetPositionRelativeInUnits(double units)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setTargetPositionRelativeInUnits -- Axis not initialized");
#endif        
        return false;
    }
      
    int32_t steps = std::round(unitsToSteps(units)); // Edited for C++

    return setTargetPositionRelativeInSteps(steps);
}

bool Axis::setTargetPositionRelativeInSteps(int32_t steps)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setTargetPositionRelativeInSteps -- Axis not initialized");
#endif        
        return false;
    }
    return setTargetPositionAbsoluteInSteps(steps + canOpenCharacteristics.x6064_positionActualValue); // Проверить
}

bool Axis::setTargetPositionAbsoluteInUnits(double units)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setTargetPositionAbsoluteInUnits -- Axis not initialized");
#endif        
        return false;
    }

    int32_t steps = std::round(unitsToSteps(units)); // Edited for C++
    return setTargetPositionAbsoluteInSteps(steps);      

}

bool Axis::setTargetPositionAbsoluteInSteps(int32_t steps)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::setTargetPositionAbsoluteInSteps -- Axis not initialized");
#endif        
        return false;
    }

    int32_t relativePosition = steps - canOpenCharacteristics.x6064_positionActualValue;
    movementUnits = stepsToUnits(relativePosition);
    movementSteps = std::fabs(relativePosition);
    canOpenCharacteristics.x607A_targetPosition = steps;
    return true;
}

double Axis::getPositionInUnits() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getPositionInUnits -- Axis not initialized");
#endif        
        return -1;
    }

    double position = stepsToUnits(canOpenCharacteristics.x6064_positionActualValue);
    return position;
}

int32_t Axis::getPositionInSteps() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getPositionInSteps -- Axis not initialized");
#endif        
        return -1;
    }

    int32_t position = canOpenCharacteristics.x6064_positionActualValue;
    return position;
}

uint32_t Axis::getStepsPerRevolution() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getStepsPerRevolution -- Axis not initialized");
#endif        
        return 0;
    }

    return stepsPerRevolution;
}

double Axis::getUnitsPerRevolution() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getUnitsPerRevolution -- Axis not initialized");
#endif        
        return -1;
    }
    
    return unitsPerRevolution;
}

double Axis::stepsToUnits(int32_t steps) const // Перевести шаги в градусы
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::stepsToUnits -- Axis not initialized");
#endif        
        return -1;
    }

    if(stepsPerRevolution == 0 || unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::stepsToUnits -- division by zero"); 
        return 0;
    }
    return steps / (double)stepsPerRevolution * unitsPerRevolution;
}

int32_t Axis::unitsToSteps(double units) const // Перевести градусы в шаги
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::unitsToSteps -- Axis not initialized");
#endif        
        return 0;
    }
    
    if(stepsPerRevolution == 0 || unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::unitsToSteps -- division by zero");
        return 0;
    }
    return units / unitsPerRevolution * stepsPerRevolution;
}

uint32_t Axis::speedUnitsToRevolutionsPerMinute(double speedUnits) const // Перевести градусы/сек в об/мин
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::speedUnitsToRevolutionsPerMinute -- Axis not initialized");
#endif        
        return 0;
    }

    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::speedUnitsToRevolutionsPerMinute -- division by zero");
        return 0;
    }
    return speedUnits * RobotConstants::Math::SECONDS_IN_MINUTE / unitsPerRevolution;
}

double Axis::revolutionsPerMinuteToSpeedUnits(uint32_t rpm) const // Перевести из об/мин в градусы/сек
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::revolutionsPerMinuteToSpeedUnits -- Axis not initialized");
#endif        
        return 0;
    }
    return (double)rpm / RobotConstants::Math::SECONDS_IN_MINUTE * unitsPerRevolution;
}

uint32_t Axis::accelerationUnitsTorpmPerSecond(double accelearionUnits) const // Перевести градусы/сек^2 в об/(мин*сек)
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::accelerationUnitsTorpmPerSecond -- Axis not initialized");
#endif        
        return 0;
    }
    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::accelerationUnitsTorpmPerSecond -- division by zero");
        return 0;
    }
    return accelearionUnits * RobotConstants::Math::SECONDS_IN_MINUTE / unitsPerRevolution;
}

double Axis::rpmPerSecondToAccelerationUnits(double rpmPerSecond) const  // Перевести об/(мин*сек) в градусы/сек^2
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::rpmPerSecondToAccelerationUnits -- Axis not initialized");
#endif        
        return 0;
    }

    if(unitsPerRevolution == 0){ // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
        Serial2.println("Axis::rpmPerSecondToAccelerationUnits -- division by zero");
        return 0;
    }   
    return (double)rpmPerSecond / RobotConstants::Math::SECONDS_IN_MINUTE * unitsPerRevolution; // TODO: потенциальная потеря точности расчетов из-за SECONDS_IN_MINUTE, лучше объявить как константу float/double
}

double Axis::getMaxLimitUnits() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getMaxLimitUnits -- Axis not initialized");
#endif        
        return -1;
    }

    return maxPositionUnits;
}

double Axis::getMinLimitUnits() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getMinLimitUnits -- Axis not initialized");
#endif        
        return -1;
    }
    return minPositionUnits;
}

uint8_t Axis::getNodeId() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getNodeId -- Axis not initialized");
#endif        
        return 0;
    }

    return nodeId;
}

double Axis::getMovementUnits() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getMovementUnits -- Axis not initialized");
#endif        
        return -1;
    }
    return movementUnits;
}

int32_t Axis::getTargetPositionAbsolute() const
{
    if (!initialized) {
#ifdef DEBUG
        Serial2.println("Axis::getTargetPositionAbsolute -- Axis not initialized");
#endif        
        return -1;
    }
    
    return canOpenCharacteristics.x607A_targetPosition;
}

}