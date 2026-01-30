#include "MoveControllerBase.h"
#include <cmath>
#include "Arduino.h"

namespace StepDirController{


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
    if (axesCnt == 0 || axes.empty()) { 
        Serial2.println("No axes configured"); 
        return; 
    }
#ifdef DEBUG
    Serial2.println("MoveControllerBase.cpp prepareMove called");
#endif    
    uint8_t maxMovementAxisId = 0;
    bool firstAxis = true;
    double maxMovement = 0;

    for (auto it = axes.begin(); it != axes.end(); ++it) {
        Axis& axis = it->second;
        double axisMovement = std::fabs(axis.getMovementUnits());
        if (firstAxis || axisMovement > maxMovement) {
            maxMovement = axisMovement;
            maxMovementAxisId = axis.nodeId;
            firstAxis = false;
        }
    }

    if(accelerationUnits == 0){ // Right now we do not support zero acceleration. But in the future we can add special handling for this case.
        Serial2.println("MoveControllerBase.cpp zero acceleration is not supported");
        return;
    }
    double tAcceleration = regularSpeedUnits / accelerationUnits; // в секундах
    if(regularSpeedUnits == 0){ // Zero speed means no movement at all. It is strange to call move with zero speed
        Serial2.println("MoveControllerBase.cpp zero regularSpeedUnits is not supported (zero speed)");
        return;
    }
    double tCruising = (maxMovement - regularSpeedUnits * regularSpeedUnits / accelerationUnits) / regularSpeedUnits;
    double res = 0;

    if(maxMovement == 0){
        Serial2.println("MoveControllerBase.cpp maxMovement division by zero. Motors do not need to move.");
        return;
    }

    if(tAcceleration == 0 || (tAcceleration + tCruising == 0)){
        Serial2.println("MoveControllerBase.cpp tAcceleration or tAcceleration + tCruising division by zero");
        return;
    }

    for (auto it = axes.begin(); it != axes.end(); ++it) {
        Axis& axis = it->second;

        double axisMovementUnits = std::fabs(axis.getMovementUnits());

        axis.acceleration = (axisMovementUnits) / (tAcceleration * (tAcceleration + tCruising));
        axis.regularSpeed = axis.acceleration * tAcceleration;
        axis.params.x6083_profileAcceleration = axis.accelerationUnitsTorpmPerSecond(axis.acceleration);
        axis.params.x6081_profileVelocity = axis.speedUnitsToRevolutionsPerMinute(axis.regularSpeed);

    }
}

void MoveControllerBase::sendMove()
{
    for (auto it = axes.begin(); it != axes.end(); ++it) {
        Axis& axis = it->second;
        canOpen->send_x6081_profileVelocity(axis.nodeId, axis.params.x6081_profileVelocity);
        delay(5);
        canOpen->send_x6083_profileAcceleration(axis.nodeId, axis.params.x6083_profileAcceleration);
        delay(5);
        
        canOpen->send_x6040_controlword(axis.nodeId, 0x004F);
        delay(5);
        
        canOpen->send_x6040_controlword(axis.nodeId, 0x005F);
        delay(5);
        
        canOpen->sendPDO4_x607A_SyncMovement(axis.nodeId, axis.getTargetPositionAbsolute());
        delay(5);
        
        // Imitation, that the motor reached the target position
        axis.setCurrentPositionInSteps(axis.params.x607A_targetPosition);
        //axis.params.x6064_positionActualValue = axis.params.x607A_targetPosition;
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
    if (axesCnt == 0) { 
        Serial2.println("MoveControllerBase.cpp start -- axesCnt is zero");
        return false;
    }   
    if (canOpen == nullptr) {
        Serial2.println("MoveControllerBase.cpp start -- canOpen is nullptr");
        return false;
    }

    this->canOpen = canOpen;
    this->axesCnt = axesCnt;

    for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId){
        axes[nodeId] = Axis(nodeId);
    }

    canOpen->setSdoReadPositionCallback([this](uint8_t nodeId, int32_t position) {
        this->positionUpdateCallback(nodeId, position);
    });



    initialized = true;
    return true;
}

void MoveControllerBase::move() {
#ifdef DEBUG
    Serial2.println("MoveControllerBase.cpp move called");
#endif 
   
    if (!initialized) {
        Serial2.println("MoveControllerBase.cpp move -- MoveControllerBase not initialized");
        return;
    }
    prepareMove();
    sendMove();

}

void MoveControllerBase::positionUpdateCallback(uint8_t nodeId, int32_t position) {
    auto it = axes.find(nodeId);
    if (it != axes.end()) {
        Axis& axis = it->second;
        
        axis.setCurrentPositionInSteps(position);
        //axis.params.x6064_positionActualValue = position;
        Serial2.println("Axis " + String(nodeId) + " position updated via SDO callback: " + String(position));
    }
}

}