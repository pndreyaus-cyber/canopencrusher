#include "MoveControllerBase.h"
#include <cmath>
#include "Arduino.h"

namespace StepDirController{
#define STEPS_PER_REVOLUTION 32768
#define UNITS_PER_REVOLUTION 7.2


void MoveControllerBase::setRegularSpeedUnits(double speed)
{
    regularSpeedUnits = std::fabs(speed); // Edited for C++
}

void MoveControllerBase::setAccelerationUnits(double acceleration)
{
    accelerationUnits = std::fabs(acceleration);  // Edited for C++
}

void MoveControllerBase::prepareMove() // TODO: Does not work for a = 0, maybe other corner cases
{
    Serial2.println("MoveControllerBase.cpp prepareMove called");
    int maxMovementAxisIndex = 0;
    for (int i = 1; i < axesCnt; ++i) {
        if (axes[i].getMovementUnits() > axes[maxMovementAxisIndex].getMovementUnits()) {
            maxMovementAxisIndex = i;
        }
    }   

    Axis *leadAxis = &axes[maxMovementAxisIndex];
    double maxPath = fabs(leadAxis->movementUnits);

    if(accelerationUnits == 0){
        Serial2.println("MoveControllerBase.cpp accelerationUnits division by zero");
    }
    double tAcceleration = regularSpeedUnits / accelerationUnits; // в секундах
    if(regularSpeedUnits == 0){
        Serial2.println("MoveControllerBase.cpp accelerationUnits division by zero");
    }
    double tCruising = (maxPath - regularSpeedUnits * regularSpeedUnits / accelerationUnits) / regularSpeedUnits;
    double res = 0;
    for (Axis& axis : axes) {
        if(maxPath == 0){
            Serial2.println("MoveControllerBase.cpp maxPath division by zero");
        }

        double syncCoefficient = fabs(axis.movementUnits) / maxPath;
        double axisMovementUnits = fabs(axis.movementUnits);
        if(tAcceleration == 0 || (tAcceleration + tCruising == 0)){
            Serial2.println("MoveControllerBase.cpp tAcceleration or tAcceleration + tCruising division by zero");
        }

        axis.acceleration = (axisMovementUnits) / (tAcceleration * (tAcceleration + tCruising));
        axis.regularSpeed = axis.acceleration * tAcceleration;
        axis.canOpenCharacteristics.x6083_profileAcceleration = axis.accelerationUnitsTorpmPerSecond(axis.acceleration);
        axis.canOpenCharacteristics.x6081_profileVelocity = axis.speedUnitsToRevolutionsPerMinute(axis.regularSpeed);

    }
}

void MoveControllerBase::sendMove()
{
    for (Axis& axis : axes) {
        canOpen->send_x6081_profileVelocity(axis.nodeId, axis.canOpenCharacteristics.x6081_profileVelocity);
        delay(5);
        canOpen->send_x6083_profileAcceleration(axis.nodeId, axis.canOpenCharacteristics.x6083_profileAcceleration);
        delay(5);
        
        canOpen->send_x6040_controlword(axis.nodeId, 0x004F);
        delay(5);
        
        canOpen->send_x6040_controlword(axis.nodeId, 0x005F);
        delay(5);
        
        canOpen->sendPDO4_x607A_SyncMovement(axis.nodeId, axis.getTargetPositionAbsolute());
        delay(5);
        
        axis.canOpenCharacteristics.x6064_positionActualValue = axis.canOpenCharacteristics.x607A_targetPosition; // Imitiation, that the motor reached the target position
    }
    
    delay(5);
    canOpen->sendSYNC();
}

double MoveControllerBase::getRegularSpeedUnits() const
{
    return regularSpeedUnits;
}

double MoveControllerBase::getAccelerationUnits() const
{
    return accelerationUnits;
}

bool MoveControllerBase::start(CanOpen* canOpen, uint8_t axesCnt){
    this->canOpen = canOpen;
    this->axesCnt = axesCnt;
    initializeAxes();
    return true;
}

void MoveControllerBase::initializeAxes(){
    axes.resize(axesCnt);

    for (uint8_t i = 0; i < axesCnt; ++i){
        axes[i] = Axis(i);
        axes[i].setStepsPerRevolution(STEPS_PER_REVOLUTION);
        axes[i].setUnitsPerRevolution(UNITS_PER_REVOLUTION);
    }
}

void MoveControllerBase::moveAbsolute() {
    Serial2.println("MoveControllerBase.cpp MoveAbsolute called");

    prepareMove();
    sendMove();

}

}