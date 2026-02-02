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
    canOpen->setElectronicGearMoleculesWriteStatusCallback([this](uint8_t nodeId, bool success) {
        this->electronicGearMoleculesWriteStatusCallback(nodeId, success);
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

void MoveControllerBase::electronicGearMoleculesWriteStatusCallback(uint8_t nodeId, bool success) {
    auto it = axes.find(nodeId);
    if (it != axes.end()) {
        
        if (success) {
            if (axisZeroInitStatus[nodeId] == ZEI_WAIT_FIRST_REPLY) {
                axisZeroInitStatus[nodeId] = ZEI_SEND_SECOND;
            } else if (axisZeroInitStatus[nodeId] == ZEI_WAIT_SECOND_REPLY) {
                axisZeroInitStatus[nodeId] = ZEI_FINISHED;
            }
            Serial2.println("Axis " + String(nodeId) + " electronic gear molecules write succeeded.");
        } else {
            axisZeroInitStatus[nodeId] = ZEI_FAILED;
            Serial2.println("Axis " + String(nodeId) + " electronic gear molecules write failed.");
        }
    }

}

void MoveControllerBase::tick() {
    if (!initialized) {
        Serial2.println("MoveControllerBase.cpp tick -- MoveControllerBase not initialized");
        return;
    }
    
    for (auto it = axes.begin(); it != axes.end(); ++it){
        Axis& axis = it->second;
        if(axisZeroInitStatus[axis.nodeId] == ZEI_SEND_FIRST) {
            if(canOpen->send_zeroInitialize(axis.nodeId, 1)){
                axisZeroInitStatus[axis.nodeId] = ZEI_WAIT_FIRST_REPLY;
            } else {
                axisZeroInitStatus[axis.nodeId] = ZEI_FAILED;
                Serial2.println("Axis " + String(axis.nodeId) + " zero initialization failed at first step");
            }
        } else if (axisZeroInitStatus[axis.nodeId] == ZEI_SEND_SECOND) {
            if(canOpen->send_zeroInitialize(axis.nodeId, 2)){
                axisZeroInitStatus[axis.nodeId] = ZEI_WAIT_SECOND_REPLY;
            } else {
                axisZeroInitStatus[axis.nodeId] = ZEI_FAILED;
                Serial2.println("Axis " + String(axis.nodeId) + " zero initialization failed at second step");
            }
        } else if (axisZeroInitStatus[axis.nodeId] == ZEI_FINISHED) {
            Serial2.println("Axis " + String(axis.nodeId) + " zero initialization finished successfully.");
            setWorkMode(axis.nodeId, 1); // Set to Profile Position Mode
            setControlWord(axis.nodeId, 0x0F); // Enable Voltage
            axisZeroInitStatus[axis.nodeId] = ZEI_NONE; // To prevent repeated messages
        } else if (axisZeroInitStatus[axis.nodeId] == ZEI_FAILED) {
            Serial2.println("Axis " + String(axis.nodeId) + " zero initialization failed.");
            axisZeroInitStatus[axis.nodeId] = ZEI_NONE; // To prevent repeated messages
        }
    }
}

void MoveControllerBase::startZeroInitialization(uint8_t nodeId) {
    if (axes.find(nodeId) == axes.end()) {
        Serial2.println("Axis " + String(nodeId) + " not found.");
        return;
    }

    if (axisZeroInitStatus.find(nodeId) == axisZeroInitStatus.end() || axisZeroInitStatus[nodeId] == ZEI_NONE) {
        Serial2.println("Starting zero initialization for Axis " + String(nodeId));
        axisZeroInitStatus[nodeId] = ZEI_SEND_FIRST;
    } else {
        Serial2.println("Zero initialization for Axis " + String(nodeId) + " is already in progress or completed.");
        return;
    }
}

void MoveControllerBase::setWorkMode(uint8_t nodeId, uint8_t mode) {
    if (!canOpen->send_x6060_modesOfOperation(nodeId, mode)) {
        Serial2.println("Failed to set work mode for Axis " + String(nodeId));
    } else {
        axes[nodeId].params.x6060_modesOfOperation = mode;
        Serial2.println("Work mode for Axis " + String(nodeId) + " set to " + String(mode));
    }
}

void MoveControllerBase::setControlWord(uint8_t nodeId, uint16_t controlWord) {
    if (!canOpen->send_x6040_controlword(nodeId, controlWord)) {
        Serial2.println("Failed to set control word for Axis " + String(nodeId));
    } else {
        axes[nodeId].params.x6040_controlword = controlWord;
        Serial2.println("Control word for Axis " + String(nodeId) + " set to " + String(controlWord, HEX));
    }
}

}