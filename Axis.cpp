#include "Axis.h"
#include <math.h>
#include <iostream>
#include "Arduino.h"

/*
Axis::Axis(const int _stepPin, const int _dirPin, const int _enablePin)
    :   stepPin(_stepPin), dirPin(_dirPin), enablePin(_enablePin)
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    if (enablePin >= 0)
        pinMode(enablePin, OUTPUT);

    setCurrentPositionInSteps(0);
}
*/
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

/*
Axis &Axis::enable()
{
    if (enablePin >= 0){
        digitalWriteFast(enablePin, !inverseEnable);
    }
    return *this;
}
*/

/*
Axis &Axis::disable()
{
    if (enablePin >= 0){
        digitalWriteFast(enablePin, inverseEnable);
    }
    return *this;
}


Axis &Axis::setInverseDirPin(bool isInverted)
{
    inverseDir = isInverted;
    return *this;
}

Axis &Axis::setInverseStepPin(bool isInverted)
{
    inverseStep = isInverted;
    return *this;
}

Axis &Axis::setInverseEnablePin(bool isInverted)
{
    inverseEnable = isInverted;
    return *this;
}
*/

Axis &Axis::setCurrentPositionInUnits(double units)
{
    int32_t steps = std::round(unitsToSteps(units)); // Edited for C++
    return setCurrentPositionInSteps(steps);
}

Axis &Axis::setCurrentPositionInSteps(int32_t steps)
{
    Serial2.println("setCurrentPositionInSteps " + String(nodeId));
    currentPosition = steps;
    Serial2.print("steps ");
    Serial2.println(steps);
    Serial2.print("currentPosition ");
    Serial2.println(currentPosition);
    Serial2.print("x6064 before: ");
    Serial2.println(canOpenCharacteristics.x6064_positionActualValue);
    canOpenCharacteristics.x6064_positionActualValue = steps;
    Serial2.print("x6064 after: ");
    Serial2.println(canOpenCharacteristics.x6064_positionActualValue);
    return *this;
}

int32_t Axis::getCurrentPositionInSteps() const
{
    return canOpenCharacteristics.x6064_positionActualValue;
}


/*
Axis &Axis::setOnStepDone(void (*callback)())
{
    onStepDone = callback;
    return *this;
}
*/

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
    Serial2.println("ENTER setTargetPositionAbsoluteInUnits " + String(nodeId));

    int32_t steps = std::round(unitsToSteps(units)); // Edited for c++
    Serial2.print("steps ");
    Serial2.println(steps);
    Serial2.print("units ");
    Serial2.println(units);
    Serial2.println("EXIT setTargetPositionAbsoluteInUnits " + String(nodeId));
    return setTargetPositionAbsoluteInSteps(steps);      

}

bool Axis::setTargetPositionAbsoluteInSteps(int32_t steps)
{
    /*
    if ((usePositionLimits) && 
        ((getPositionInSteps() + steps > std::round(unitsToSteps(maxPositionUnits))) || 
            (getPositionInSteps() + steps < std::round(unitsToSteps(minPositionUnits))))) // Edited for C++ (std)
        return false;
    */
    Serial2.println("ENTER setTargetPositionAbsoluteInSteps " + String(nodeId));
    Serial2.print("6064_positionActualValue ");
    Serial2.println(canOpenCharacteristics.x6064_positionActualValue);
    Serial2.print("steps ");
    Serial2.println(steps);
    int32_t relativePosition = steps - canOpenCharacteristics.x6064_positionActualValue;
    movementUnits = stepsToUnits(relativePosition);
    Serial2.print("movementUnits ");
    Serial2.println(movementUnits);
    movementSteps = std::fabs(relativePosition);
    Serial2.print("movementSteps");
    Serial2.println(movementSteps);
    Serial2.println("movementUnits for " + String(nodeId) + ": " + String(movementUnits));
    canOpenCharacteristics.x607A_targetPosition = steps;
    Serial2.println("607A after ");
    Serial2.println(canOpenCharacteristics.x607A_targetPosition);
    Serial2.println("EXIT setTargetPositionAbsoluteInSteps " + String(nodeId));
    return true;
}

double Axis::getPositionInUnits() const
{
    //noInterrupts(); // Edited for C++
    double position = stepsToUnits(canOpenCharacteristics.x6064_positionActualValue);
    //interrupts(); // Edited for C++
    return position;
}

int32_t Axis::getPositionInSteps() const
{
    // noInterrupts(); // Edited for C++
    int32_t position = canOpenCharacteristics.x6064_positionActualValue;
    //interrupts(); // Edited for C++
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
    if(stepsPerRevolution == 0 || unitsPerRevolution == 0){
        Serial2.println("Axis::stepsToUnits -- division by zero");
        return 0;
    }
    return steps / (double)stepsPerRevolution * unitsPerRevolution;
}

int32_t Axis::unitsToSteps(double units) const // Перевести градусы в шаги
{
    if(stepsPerRevolution == 0 || unitsPerRevolution == 0){
        Serial2.println("Axis::unitsToSteps -- division by zero");
        return 0;
    }
    return units / unitsPerRevolution * stepsPerRevolution;
}

uint32_t Axis::speedUnitsToRevolutionsPerMinute(double speedUnits) const // Перевести градусы/сек в об/мин
{
    if(unitsPerRevolution == 0){
        Serial2.println("Axis::speedUnitsToRevolutionsPerMinute -- division by zero");
        return 0;
    }
    return speedUnits * SECONDS_IN_MINUTE / unitsPerRevolution;
}

double Axis::revolutionsPerMinuteToSpeedUnits(uint32_t rpm) const // Перевести из об/мин в градусы/сек
{
    if(unitsPerRevolution == 0){
        Serial2.println("Axis::revolutionsPerMinuteToSpeedUnits -- division by zero");
        return 0;
    }
    return (double)rpm / SECONDS_IN_MINUTE * unitsPerRevolution;
}

uint32_t Axis::accelerationUnitsTorpmPerSecond(double accelearionUnits) const // Перевести градусы/сек^2 в об/(мин*сек)
{
    if(unitsPerRevolution == 0){
        Serial2.println("Axis::accelerationUnitsTorpmPerSecond -- division by zero");
        return 0;
    }
    return accelearionUnits * SECONDS_IN_MINUTE / unitsPerRevolution;
}

double Axis::rpmPerSecondToAccelerationUnits(double rpmPerSecond) const  // Перевести об/(мин*сек) в градусы/сек^2
{
    if(unitsPerRevolution == 0){
        Serial2.println("Axis::rpmPerSecondToAccelerationUnits -- division by zero");
        return 0;
    }   
    return (double)rpmPerSecond / SECONDS_IN_MINUTE * unitsPerRevolution;
}

/*
Axis &Axis::setDirection(Direction newDirection)
{
    direction = newDirection;
    uint8_t dir = direction == Direction::POSITIVE ? !inverseDir : inverseDir;
    digitalWriteFast(dirPin, dir);
    return *this;
}
*/
/*
Axis &Axis::stepOn()
{
    digitalWriteFast(stepPin, !inverseStep);
    return *this;
}
*/
/*
Axis &Axis::stepOff()
{
    digitalWriteFast(stepPin, inverseStep);
    return *this;
}
*/
/*
Axis & Axis::stepDone()
{
    //direction == Direction::POSITIVE ? currentPosition++ : currentPosition--;
    //stepsDone++;
    if (onStepDone != nullptr)
        onStepDone();
        
    return *this;
}
*/

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