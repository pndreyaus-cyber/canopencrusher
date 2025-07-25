#include "MoveControllerBase.h"
#include <cmath>
#include "Arduino.h"

namespace StepDirController{
MoveControllerBase::MoveControllerBase(MyCanOpen *canOpen) : canOpen(canOpen)
{

}

void MoveControllerBase::setRegularSpeedUnits(double speed)
{
    speed = std::fabs(speed); // Edited for C++
    //regularSpeedUnits = speed > jerkSpeedUnits ? speed : jerkSpeedUnits;
    regularSpeedUnits = speed;
}

void MoveControllerBase::setAccelerationUnits(double acceleration)
{
    accelerationUnits = std::fabs(acceleration);  // Edited for C++
}

void MoveControllerBase::setOnMoveStarted(void (*callback)())
{
    onMoveStarted = callback;
}

void MoveControllerBase::setOnMoveFinished(void (*callback)())
{
    onMoveFinished = callback;
}

void MoveControllerBase::setOnEmergensyStoped(void (*callback)())
{
    onEmergensyStoped = callback;
}


void MoveControllerBase::prepareMoveWithoutSync()
{
    Axis** axes = axisList;
    while(*(axes) != nullptr)
    {
        (*axes)->canOpenCharacteristics.x6081_profileVelocity = (*axes)->speedUnitsToRevolutionsPerMinute(regularSpeedUnits);
        (*axes)->canOpenCharacteristics.x6083_profileAcceleration = (*axes)->accelerationUnitsTorpmPerSecond(accelerationUnits);
        (*axes++);
    }
}

ResultParams MoveControllerBase::prepareMove() // TODO: Does not work for a = 0, maybe otehr corner cases
{
    leadAxis = axisList[0];

    if (leadAxis == nullptr)
        return ResultParams();

    stepsDoneAll = 0;
    stepsAll = 0;

    Axis** axes = axisList;
    while (*(axes) != nullptr)
    {
        if (std::fabs(leadAxis->movementUnits) < std::fabs((*axes)->movementUnits)) // Edited for C++
            leadAxis = (*axes);

        (*axes++);
    }
    

    ResultParams p;
    double maxPath = fabs(leadAxis->movementUnits);
    axes = axisList;
    if(accelerationUnits == 0){
        Serial2.println("MoveControllerBase.cpp accelerationUnits division by zero");
    }
    double tAcceleration = regularSpeedUnits / accelerationUnits; // в секундах
    if(regularSpeedUnits == 0){
        Serial2.println("MoveControllerBase.cpp accelerationUnits division by zero");
    }
    double tCruising = (maxPath - regularSpeedUnits * regularSpeedUnits / accelerationUnits) / regularSpeedUnits;
    double res = 0;
    p.maxPath = maxPath;
    p.tAcceleration = tAcceleration;
    p.tCruising = tCruising;
    int i = 0;
    while (*(axes) != nullptr)
    {
        if(maxPath == 0){
            Serial2.println("MoveControllerBase.cpp maxPath division by zero");
       }
        double syncCoefficient = fabs((*axes)->movementUnits) / maxPath;
        double axisMovementUnits = fabs((*axes)->movementUnits);
        if(tAcceleration == 0 || (tAcceleration + tCruising == 0)){
            Serial2.println("MoveControllerBase.cpp tAcceleration or tAcceleration + tCruising division by zero");
        }
        (*axes)->acceleration = (axisMovementUnits) / (tAcceleration * (tAcceleration + tCruising));
        (*axes)->regularSpeed = (*axes)->acceleration * tAcceleration;
        (*axes)->canOpenCharacteristics.x6083_profileAcceleration = (*axes)->accelerationUnitsTorpmPerSecond((*axes)->acceleration);
        (*axes)->canOpenCharacteristics.x6081_profileVelocity = (*axes)->speedUnitsToRevolutionsPerMinute((*axes)->regularSpeed);
        //(*axes)->canOpenCharacteristics.x6081_profileVelocity = 0.12321;
        p.movementUnits[i] = (*axes)->movementUnits;
        p.syncCoefficient[i] = syncCoefficient;
        p.acceleration[i] = (*axes)->acceleration;
        p.regularSpeed[i] = (*axes)->regularSpeed;
        p.speed[i] = (*axes)->canOpenCharacteristics.x6081_profileVelocity;
        p.testValue[i] = (*axes)->canOpenCharacteristics.x6083_profileAcceleration;
        i++;

        (*axes++);
    }

    axes = axisList;
    //startGlobalTime = micros();
    /*
    while (*(axes) != nullptr)
    {
        (*axes++)->startStepTime = startGlobalTime;
    }
    */
    Serial2.println("PREPAREMOVEOK");
    return p;
}

void MoveControllerBase::sendMove()
{
    Axis** axes = axisList;
    Serial2.println("sendMove()");
    while (*(axes) != nullptr)
    {
        Axis* axis = *axes;
        canOpen->send_x6081_profileVelocity(axis->nodeId, axis->canOpenCharacteristics.x6081_profileVelocity);
        delay(5);
        canOpen->send_x6083_profileAcceleration(axis->nodeId, axis->canOpenCharacteristics.x6083_profileAcceleration);
        delay(5);
        
        canOpen->send_x6040_controlword(axis->nodeId, 0x004F);
        delay(5);
        
        canOpen->send_x6040_controlword(axis->nodeId, 0x005F);
        delay(5);
        
        canOpen->sendPDO4_x607A_SyncMovement(axis->nodeId, axis->getTargetPositionAbsolute());
        delay(5);
        
        (*axes)->canOpenCharacteristics.x6064_positionActualValue = (*axes)->canOpenCharacteristics.x607A_targetPosition;
        
        (*axes++);
    }
    delay(5);
    Serial2.println("SYNC before");
    canOpen->sendSYNC();
    Serial2.println("SYNC aftere");


}

double MoveControllerBase::getRegularSpeedUnits() const
{
    return regularSpeedUnits;
}

double MoveControllerBase::getAccelerationUnits() const
{
    return accelerationUnits;
}

}