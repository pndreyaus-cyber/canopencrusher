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
        
        canOpen->setPositionActualValueCallback_0x6064([this](uint8_t nodeId, bool success, int32_t position) {
            this->regularPositionActualValueCallback(nodeId, success, position);
        }, nodeId);
    }

    canOpen->setHeartbeatCallback([this](uint8_t nodeId, uint8_t status) {
        this->regularHeartbeatCallback(nodeId, status);
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

void MoveControllerBase::positionUpdate(uint8_t nodeId, int32_t position) {
    auto it = axes.find(nodeId);
    if (it != axes.end()) {
        Axis& axis = it->second;
        
        axis.setCurrentPositionInSteps(position);
        //axis.params.x6064_positionActualValue = position;
        Serial2.println("Axis " + String(nodeId) + " position updated via SDO callback: " + String(position));
    }
}

void MoveControllerBase::startZeroInitializationAllAxes() {
    zeroInitializeSingleAxis = false;
    for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId) {
        startZeroInitializationSingleAxis(nodeId);
    }
}

void MoveControllerBase::startZeroInitializationSingleAxis(uint8_t nodeId) {
    if (axes[nodeId].initStatus == ZEI_ONGOING || axes[nodeId].initStatus == ZEI_FINISHED) {
        Serial2.println("Zero initialization already in progress or finished for Axis " + String(nodeId));
        return;
    }
    axes[nodeId].initStatus = ZEI_ONGOING;
    if (zeroInitializeSingleAxis) {
        axisToInitialize = nodeId;
    }
    
    canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A([this](uint8_t callbackNodeId, bool success) {
        this->zeroInitialize_firstWriteTo_0x260A(callbackNodeId, success);
    }, nodeId);

    if (!canOpen->send_x260A_electronicGearMolecules(nodeId, 0xEA66)) {
        Serial2.println("Failed to start zero initialization for Axis " + String(nodeId));
        axes[nodeId].initStatus = ZEI_FAILED;
        zeroInitialize_finalResult();
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

// Callbacks 
void MoveControllerBase::zeroInitialize_firstWriteTo_0x260A(uint8_t nodeId, bool success) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization phase 1 failed for Axis ",
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, callbackNodeId);
        },
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A([this](uint8_t cbNodeId, bool cbSuccess) {
                this->zeroInitialize_secondWriteTo_0x260A(cbNodeId, cbSuccess);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->send_x260A_electronicGearMolecules(callbackNodeId, 0xEA70);
        },
        "Failed to send second write for zero initialization for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_secondWriteTo_0x260A(uint8_t nodeId, bool success) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization phase 2 failed for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, callbackNodeId);
        },
        nullptr,
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setControlWordWriteStatusCallback_0x6040([this](uint8_t cbNodeId, bool cbSuccess) {
                this->zeroInitialize_firstWriteTo_0x6040(cbNodeId, cbSuccess);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->send_x6040_controlword(callbackNodeId, 0x000F);
        },
        "Failed to send control word for zero initialization for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_firstWriteTo_0x6040(uint8_t nodeId, bool success) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization control word write failed for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, callbackNodeId);
        },
        nullptr,
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setModesOfOperationWriteStatusCallback_0x6060([this](uint8_t cbNodeId, bool cbSuccess) {
                this->zeroInitialize_writeTo_0x6060(cbNodeId, cbSuccess);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->send_x6060_modesOfOperation(callbackNodeId, 0x01);
        },
        "Failed to send modes of operation for zero initialization for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setModesOfOperationWriteStatusCallback_0x6060(nullptr, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_writeTo_0x6060(uint8_t nodeId, bool success) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization modes of operation write failed for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setModesOfOperationWriteStatusCallback_0x6060(nullptr, callbackNodeId);
        },
        nullptr,
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setPositionActualValueCallback_0x6064([this](uint8_t cbNodeId, bool cbSuccess, int32_t position) {
                this->zeroInitialize_requestPosition_0x6064(cbNodeId, cbSuccess, position);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->sendSDORead(callbackNodeId, RobotConstants::ODIndices::POSITION_ACTUAL_VALUE, 0x00);
        },
        "Failed to request position for zero initialization for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setPositionActualValueCallback_0x6064([this](uint8_t cbNodeId, bool cbSuccess, int32_t position) {
                this->regularPositionActualValueCallback(cbNodeId, cbSuccess, position);
            }, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_requestPosition_0x6064(uint8_t nodeId, bool success, int32_t position) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization position read failed for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setPositionActualValueCallback_0x6064([this](uint8_t cbNodeId, bool cbSuccess, int32_t position) {
                this->regularPositionActualValueCallback(cbNodeId, cbSuccess, position);
            }, callbackNodeId);
        },
        nullptr,
        [this, position](uint8_t callbackNodeId) {
            this->positionUpdate(callbackNodeId, position);
        },
        [this](uint8_t callbackNodeId) {
            this->canOpen->setControlWordWriteStatusCallback_0x6040([this](uint8_t cbNodeId, bool cbSuccess) {
                this->zeroInitialize_secondWriteTo_0x6040(cbNodeId, cbSuccess);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->send_x6040_controlword(callbackNodeId, 0x002F);
        },
        "Failed to send modes of operation for zero initialization for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_secondWriteTo_0x6040(uint8_t nodeId, bool success) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization final control word write failed for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, callbackNodeId);
        },
        nullptr,
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setTargetPositionWriteStatusCallback_0x607A([this](uint8_t cbNodeId, bool cbSuccess) {
                this->zeroInitialize_writeTo_0x607A(cbNodeId, cbSuccess);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->send_x607A_targetPosition(callbackNodeId, this->axes[callbackNodeId].getCurrentPositionInSteps());
        },
        "Failed to send target position for zero initialization for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setTargetPositionWriteStatusCallback_0x607A(nullptr, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_writeTo_0x607A(uint8_t nodeId, bool success) {
    zeroInitialize_advanceStep(
        nodeId,
        success,
        "Zero initialization target position write failed for Axis ",
        [this](uint8_t callbackNodeId) {
            this->canOpen->setTargetPositionWriteStatusCallback_0x607A(nullptr, callbackNodeId);
        },
        nullptr,
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setStatusWordCallback_0x6041([this](uint8_t cbNodeId, bool cbSuccess, uint16_t statusWord) {
                this->zeroInitialize_requestStatusword_0x6041(cbNodeId, cbSuccess, statusWord);
            }, callbackNodeId);
        },
        [this](uint8_t callbackNodeId) {
            return this->canOpen->sendSDORead(callbackNodeId, RobotConstants::ODIndices::STATUSWORD, 0x00);
        },
        nullptr,
        [this](uint8_t callbackNodeId) {
            this->canOpen->setStatusWordCallback_0x6041(nullptr, callbackNodeId);
        });
}

void MoveControllerBase::zeroInitialize_requestStatusword_0x6041(uint8_t nodeId, bool success, uint16_t statusWord) {
    canOpen->setStatusWordCallback_0x6041(nullptr, nodeId);

    if(!success) {
        Serial2.println("Zero initialization status word read failed for Axis " + String(nodeId));
        axes[nodeId].initStatus = ZEI_FAILED;
        zeroInitialize_finalResult();
        return;
    }

    bool targetReached = statusWord & 0b000001000000000;
    if (!targetReached) {
        Serial2.println("Zero initialization failed: target not reached for Axis " + String(nodeId));
        axes[nodeId].initStatus = ZEI_FAILED;
    } else {
        axes[nodeId].initStatus = ZEI_FINISHED;
    }
    zeroInitialize_finalResult();
}

void MoveControllerBase::zeroInitialize_finalResult() {
    if (zeroInitializeSingleAxis) {
        String status;
        if (axes[axisToInitialize].initStatus == ZEI_FINISHED) {
            status = "successful";
        } else if (axes[axisToInitialize].initStatus == ZEI_FAILED) {
            status = "failed";
        } else {
            status = "ERROR IN CODE!";
        }
        Serial2.println("Zero initialization for Axis " + String(axisToInitialize) + " " + status);
        axisToInitialize = 0;
        return;
    }
    
    String successfullAxes = "";
    String failedAxes = ""; 
    for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId) {
        if (axes[nodeId].initStatus == ZEI_ONGOING) {
            return; // Still ongoing for some axes
        } else if (axes[nodeId].initStatus == ZEI_FINISHED) {
            successfullAxes += String(nodeId) + " ";
        } else if (axes[nodeId].initStatus == ZEI_FAILED) {
            failedAxes += String(nodeId) + " ";
        }
    }
    Serial2.println("Zero initialization process completed. Successful axes: " + successfullAxes + ". Failed axes: " + failedAxes);
}


void MoveControllerBase::regularHeartbeatCallback(uint8_t nodeId, uint8_t status) {
    String statusStr;
    if (status == 0x05) {
        statusStr = "operational";
    } else if (status == 0x04) {
        statusStr = "alarm";
    } else if (status == 0x7F) {
        statusStr = "pre-operational";
    } else if (status == 0x00) {
        statusStr = "boot-up";
    } else{
        statusStr = "unknown";
    }        
    //Serial2.println("Heartbeat " + String(node) + ": " + status);
}

void MoveControllerBase::regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position) {
    if (!success) {
        Serial2.println("Failed to read Position Actual Value for node " + String(nodeId));
        return;
    }

    Serial2.print("Position Actual Value from node ");
    Serial2.print(nodeId);
    Serial2.print(": ");
    Serial2.println(position, HEX);
    
    positionUpdate(nodeId, position);
}