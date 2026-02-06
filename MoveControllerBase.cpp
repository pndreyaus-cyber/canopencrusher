#include <cmath>
#include "MoveControllerBase.h"
#include "Arduino.h"

namespace StepDirController
{
    void MoveControllerBase::setRegularSpeedUnits(double speed)
    {
        regularSpeedUnits = std::fabs(speed); // Edited for C++
    }

    void MoveControllerBase::setAccelerationUnits(double acceleration)
    {
        accelerationUnits = std::fabs(acceleration); // Edited for C++
    }

    void MoveControllerBase::prepareMove() // TODO: Does not work for a = 0, maybe other corner cases
    {
        if (axesCnt == 0 || axes.empty())
        {
            addDataToOutQueue("No axes configured");
            return;
        }
#ifdef DEBUG
        addDataToOutQueue("MoveControllerBase.cpp prepareMove called");
#endif
        uint8_t maxMovementAxisId = 0;
        bool firstAxis = true;
        double maxMovement = 0;

        for (auto it = axes.begin(); it != axes.end(); ++it)
        {
            Axis &axis = it->second;
            double axisMovement = std::fabs(axis.getMovementUnits());
            if (firstAxis || axisMovement > maxMovement)
            {
                maxMovement = axisMovement;
                maxMovementAxisId = axis.nodeId;
                firstAxis = false;
            }
        }

        if (accelerationUnits == 0)
        { // Right now we do not support zero acceleration. But in the future we can add special handling for this case.
            addDataToOutQueue("MoveControllerBase.cpp zero acceleration is not supported");
            return;
        }
        double tAcceleration = regularSpeedUnits / accelerationUnits; // в секундах
        if (regularSpeedUnits == 0)
        { // Zero speed means no movement at all. It is strange to call move with zero speed
            addDataToOutQueue("MoveControllerBase.cpp zero regularSpeedUnits is not supported (zero speed)");
            return;
        }
        double tCruising = (maxMovement - regularSpeedUnits * regularSpeedUnits / accelerationUnits) / regularSpeedUnits;
        double res = 0;

        if (maxMovement == 0)
        {
            addDataToOutQueue("MoveControllerBase.cpp maxMovement division by zero. Motors do not need to move.");
            return;
        }

        if (tAcceleration == 0 || (tAcceleration + tCruising == 0))
        {
            addDataToOutQueue("MoveControllerBase.cpp tAcceleration or tAcceleration + tCruising division by zero");
            return;
        }

        for (auto it = axes.begin(); it != axes.end(); ++it)
        {
            Axis &axis = it->second;

            double axisMovementUnits = std::fabs(axis.getMovementUnits());

            axis.acceleration = (axisMovementUnits) / (tAcceleration * (tAcceleration + tCruising));
            axis.regularSpeed = axis.acceleration * tAcceleration;
            axis.params.x6083_profileAcceleration = axis.accelerationUnitsTorpmPerSecond(axis.acceleration);
            axis.params.x6081_profileVelocity = axis.speedUnitsToRevolutionsPerMinute(axis.regularSpeed);
        }
    }

    void MoveControllerBase::sendMove()
    {
        for (auto it = axes.begin(); it != axes.end(); ++it)
        {
            Axis &axis = it->second;
            // canOpen->send_x6081_profileVelocity(axis.nodeId, axis.params.x6081_profileVelocity);
            // delay(5);
            // canOpen->send_x6083_profileAcceleration(axis.nodeId, axis.params.x6083_profileAcceleration);
            // delay(5);

            canOpen->send_x6040_controlword(axis.nodeId,
                                            0x004F);    
            delay(5);

            // canOpen->send_x6040_controlword(axis.nodeId,
            //                                 0x005F);
            // delay(5);

            // canOpen->sendPDO4_x607A_SyncMovement(axis.nodeId, axis.getTargetPositionAbsolute());
            // delay(5);

            // Imitation, that the motor reached the target position
            axis.setCurrentPositionInSteps(axis.params.x607A_targetPosition);
            // axis.params.x6064_positionActualValue = axis.params.x607A_targetPosition;
        }

        delay(5);
        //canOpen->sendSYNC();
    }

    double MoveControllerBase::getRegularSpeedUnits() const
    {
        return regularSpeedUnits;
    }

    double MoveControllerBase::getAccelerationUnits() const
    {
        return accelerationUnits;
    }

    bool MoveControllerBase::start(CanOpen *canOpen, uint8_t axesCnt)
    {
        if (axesCnt == 0)
        {
            addDataToOutQueue("MoveControllerBase.cpp start -- axesCnt is zero");
            return false;
        }
        if (canOpen == nullptr)
        {
            addDataToOutQueue("MoveControllerBase.cpp start -- canOpen is nullptr");
            return false;
        }

        this->canOpen = canOpen;
        this->axesCnt = axesCnt;

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            axes[nodeId] = Axis(nodeId);
            axes[nodeId].lastHeartbeatMs = 0;
            axes[nodeId].isAlive = false;

            setRegularPositionActualValueCallback(nodeId);
        }

        canOpen->setHeartbeatCallback([this](uint8_t nodeId, uint8_t status)
                                      { this->regularHeartbeatCallback(nodeId, status); });

        initialized = true;
        Serial2.println("MoveControllerBase initialized with " + String(axesCnt) + " axes." + String(axes.size()));
        return true;
    }

    void MoveControllerBase::tick()
    {
        if (!initialized)
        {
            return;
        }
        checkTimeouts();

        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis &axis = axes[nodeId];
            if(axis.initStatus == RobotConstants::InitStatus::ZEI_ONGOING && !axis.isAlive)
            {
#ifdef DEBUG
                addDataToOutQueue("Zero Initialization failed for Axis " + String(nodeId) + ": Heartbeat timeout");
#endif
                if(nodeId == 2){
                    axis.initStatus = RobotConstants::InitStatus::ZEI_FAILED;  
                }
                //axis.initStatus = RobotConstants::InitStatus::ZEI_FAILED;
                // zeroInitialize_finalResult();
            }
        }
    }

    void MoveControllerBase::move()
    {
#ifdef DEBUG
        addDataToOutQueue("MoveControllerBase.cpp move called");
#endif

        if (!initialized)
        {
            addDataToOutQueue("MoveControllerBase.cpp move -- MoveControllerBase not initialized");
            return;
        }
        prepareMove();
        sendMove();
    }

    void MoveControllerBase::positionUpdate(uint8_t nodeId, int32_t position)
    {
        auto it = axes.find(nodeId);
        if (it != axes.end())
        {
            Axis &axis = it->second;

            axis.setCurrentPositionInSteps(position);
            // axis.params.x6064_positionActualValue = position;
            //addDataToOutQueue("Axis " + String(nodeId) + " new position: " + String(position) + "; " + String(position, HEX) + "; " + String(position, BIN));
        }
    }

    void MoveControllerBase::setRegularPositionActualValueCallback(uint8_t nodeId)
    {
        canOpen->setPositionActualValueCallback_0x6064([this](uint8_t callbackNodeId, bool success, int32_t position)
                                                       { this->regularPositionActualValueCallback(callbackNodeId, success, position); }, nodeId);
    }

    void MoveControllerBase::startZeroInitializationAllAxes()
    {
        zeroInitializeSingleAxis = false;
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            startZeroInitializationSingleAxis(nodeId);
            delay(1000);
        }
    }

    void MoveControllerBase::startZeroInitializationSingleAxis(uint8_t nodeId)
    {
        axes[nodeId].statuswordReadAttempts = 0;
        
        if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_ONGOING || axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_FINISHED)
        {
            addDataToOutQueue("Zero initialization already in progress or finished for Axis " + String(nodeId));
            return;
        }
        zeroInitialize_start(nodeId);
    }

    void MoveControllerBase::setWorkMode(uint8_t nodeId, uint8_t mode)
    {
        if (!canOpen->send_x6060_modesOfOperation(nodeId, mode))
        {
            addDataToOutQueue("Failed to set work mode for Axis " + String(nodeId));
        }
        else
        {
            axes[nodeId].params.x6060_modesOfOperation = mode;
            addDataToOutQueue("Work mode for Axis " + String(nodeId) + " set to " + String(mode));
        }
    }

    void MoveControllerBase::setControlWord(uint8_t nodeId, uint16_t controlWord)
    {
        if (!canOpen->send_x6040_controlword(nodeId, controlWord))
        {
            addDataToOutQueue("Failed to set control word for Axis " + String(nodeId));
        }
        else
        {
            axes[nodeId].params.x6040_controlword = controlWord;
            addDataToOutQueue("Control word for Axis " + String(nodeId) + " set to " + String(controlWord, HEX));
        }
    }

    bool MoveControllerBase::checkResponseStatus(uint8_t nodeId, bool success, String errorMessage)
    {
        if (!success)
        {
#ifdef DEBUG
            addDataToOutQueue("ZEI Failed for Axis " + String(nodeId) + ": " + errorMessage);
#endif
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FAILED;
            zeroInitialize_finalResult();
        }
        return success;
    }
    // Callbacks
    /*
    Acknowldegement callbacks should have this structure:
    1) Set the current callback to either nullptr or to the regular callback (if applicable)
    2) Check the response status using checkResponseStatus(). If it fails, return from the function
    3) Set the next callback
    4) Send the next command
    5) Pass status of sending to checkResponseStatus()
    6) If sending fails, set the next callback to nullptr or regular callback (if applicable)

    Read response callbacks should have this structure:
    1) Set the current callback to either nullptr or to the regular callback (if applicable
    2) Check the response status using checkResponseStatus(). If it fails, return from the function
    3) Process received data
    4) Set the next callback
    5) Send the next command
    6) Pass status of sending to checkResponseStatus()
    7) If sending fails, set the next callback to nullptr or regular callback (if applicable)

    So read response callbacks differ from acknowldegement callbacks by steps 3 (process data) only.


    In the end of the sequence, you can insert a funneling function, which will track if every axis has finished and report the final result
*/

    void MoveControllerBase::zeroInitialize_start(uint8_t nodeId)
    {
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_ONGOING;
        axes[nodeId].statuswordReadAttempts = 0;
        if (zeroInitializeSingleAxis)
        {
            axisToInitialize = nodeId;
        }


        {
            //addDataToOutQueue("Failed to start zero initialization for Axis " + String(nodeId));
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FAILED;
            zeroInitialize_finalResult();
        }
    }

    void MoveControllerBase::zeroInitialize_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 5
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A([this](uint8_t callbackNodeId, bool success)
                                          { this->zeroInitialize_AfterFirstWriteTo_0x260A(callbackNodeId, success); }, nodeId);

        //addDataToOutQueue("Starting zero initialization for Axis " + String(nodeId));
        if (!canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                         0xEA66))
    }

    void MoveControllerBase::zeroInitialize_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Zero initialization phase 0 failed"))
        {
            return;
        }
        // Step 3
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A([this](uint8_t cbNodeId, bool cbSuccess)
                                          { this->zeroInitialize_AfterSecondWriteTo_0x260A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x260A_electronicGearMolecules(nodeId,
                                                                       0xEA70);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "Failed to send electronic gear molecules for zero initialization"))
        {
            // Step 6
            canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        }

    }

    void MoveControllerBase::zeroInitialize_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setElectronicGearMoleculesWriteStatusCallback_0x260A(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Zero initialization phase 1 failed"))
        {
            return;
        }
        //axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FINISHED;
        //zeroInitialize_finalResult();
        
        // Step 3
        canOpen->setControlWordWriteStatusCallback_0x6040([this](uint8_t cbNodeId, bool cbSuccess)
                                  { this->zeroInitialize_AfterFirstWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x000F);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "Failed to send control word for zero initialization"))
        {
            // Step 6
            canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        }
        
    }

    void MoveControllerBase::zeroInitialize_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Failed to write 0x6040"))
        {
            return;
        }
        axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FINISHED;
        zeroInitialize_finalResult();
        /*
        // Step 3
        canOpen->setModesOfOperationWriteStatusCallback_0x6060([this](uint8_t cbNodeId, bool cbSuccess)
                                       { this->zeroInitialize_AfterWriteTo_0x6060(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x6060_modesOfOperation(nodeId,
                                                                0x01);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "Failed to write to 0x6060"))
        {
            // Step 6
            canOpen->setModesOfOperationWriteStatusCallback_0x6060(nullptr, nodeId);
        }
        */
    }

    void MoveControllerBase::zeroInitialize_AfterWriteTo_0x6060(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setModesOfOperationWriteStatusCallback_0x6060(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Write to 0x6060 failed"))
        {
            return;
        }
        // Step 3
        canOpen->setPositionActualValueCallback_0x6064([this](uint8_t cbNodeId, bool cbSuccess, int32_t position)
                                   { this->zeroInitialize_AfterReadPosition_0x6064(cbNodeId, cbSuccess, position); }, nodeId);
        // Step 4

        bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::POSITION_ACTUAL_VALUE,
                                                0x00);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "Send to 0x6064 failed"))
        {
            // Step 6
            setRegularPositionActualValueCallback(nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterReadPosition_0x6064(uint8_t nodeId, bool success, int32_t position)
    {
        // Step 1
        setRegularPositionActualValueCallback(nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Read from 0x6064 failed"))
        {
            return;
        }
        // Step 3 (processing data)
        positionUpdate(nodeId, position);
        // Step 4
        canOpen->setControlWordWriteStatusCallback_0x6040([this](uint8_t cbNodeId, bool cbSuccess)
                                  { this->zeroInitialize_AfterSecondWriteTo_0x6040(cbNodeId, cbSuccess); }, nodeId);
        // Step 5
        bool successSend = canOpen->send_x6040_controlword(nodeId,
                                                           0x004F);
        // Step 6
        if (!checkResponseStatus(nodeId, successSend,
                                 "Failed to send second 0x6040"))
        {
            // Step 7
            canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setControlWordWriteStatusCallback_0x6040(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Failed to write second 0x6040"))
        {
            return;
        }
        // Step 3
        canOpen->setTargetPositionWriteStatusCallback_0x607A([this](uint8_t cbNodeId, bool cbSuccess)
                                     { this->zeroInitialize_AfterWriteTo_0x607A(cbNodeId, cbSuccess); }, nodeId);
        // Step 4
        bool successSend = canOpen->send_x607A_targetPosition(nodeId, axes[nodeId].getCurrentPositionInSteps());
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "Failed to write 0x607A"))
        {
            // Step 6
            canOpen->setTargetPositionWriteStatusCallback_0x607A(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterWriteTo_0x607A(uint8_t nodeId, bool success)
    {
        // Step 1
        canOpen->setTargetPositionWriteStatusCallback_0x607A(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Write to 0x607A failed"))
        {
            return;
        }
        // Step 3
        canOpen->setStatusWordCallback_0x6041([this](uint8_t cbNodeId, bool cbSuccess, uint16_t statusWord)
                              { this->zeroInitialize_AfterReadStatusword_0x6041(cbNodeId, cbSuccess, statusWord); }, nodeId);
        // Step 4
        delay(1000); // TODO: Do something with this delay, maybe wait for SYNC or something else
        bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::STATUSWORD,
                                                0x00);
        // Step 5
        if (!checkResponseStatus(nodeId, successSend,
                                 "Send to 0x6041 failed"))
        {
            // Step 6
            canOpen->setStatusWordCallback_0x6041(nullptr, nodeId);
        }
    }

    void MoveControllerBase::zeroInitialize_AfterReadStatusword_0x6041(uint8_t nodeId, bool success, uint16_t statusWord)
    {
        // Step 1
        canOpen->setStatusWordCallback_0x6041(nullptr, nodeId);
        // Step 2
        if (!checkResponseStatus(nodeId, success,
                                 "Read from 0x6041 failed"))
        {
            return;
        }
        // Step 3 (processing data)
#ifdef DEBUG
        addDataToOutQueue("Statusword from Axis " + String(nodeId) + ": " + String(statusWord, HEX) + " " + String(statusWord, BIN));
#endif        
        bool targetReached = statusWord & 0b000010000000000;
        axes[nodeId].statuswordReadAttempts++;
        if (!targetReached && axes[nodeId].statuswordReadAttempts < RobotConstants::Robot::ZEI_MAX_STATUSWORD_READ_ATTEMPTS)
        {
            canOpen->setStatusWordCallback_0x6041([this](uint8_t cbNodeId, bool cbSuccess, uint16_t cbStatusWord)
                                                  { this->zeroInitialize_AfterReadStatusword_0x6041(cbNodeId, cbSuccess, cbStatusWord); }, nodeId);
            delay(10); // TODO: Do something with this delay, maybe wait for SYNC or something else
            bool successSend = canOpen->sendSDORead(nodeId, RobotConstants::ODIndices::STATUSWORD,
                                                    0x00);
            if (!checkResponseStatus(nodeId, successSend,
                                     "Send to 0x6041 failed" + String(axes[nodeId].statuswordReadAttempts)))
            {
                canOpen->setStatusWordCallback_0x6041(nullptr, nodeId);
            }
            return;
        }

        if (!targetReached)
        {
#ifdef DEBUG
            String message = "Target not reached! " + String(nodeId);
            addDataToOutQueue(message);
#endif
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FAILED;
        }
        else
        {
            axes[nodeId].initStatus = RobotConstants::InitStatus::ZEI_FINISHED;
        }
        // Steps 4, 5, 6, and 7 -- not applicable, because it is the end of the sequence
        // Call funneling function
        zeroInitialize_finalResult();
    }

    void MoveControllerBase::zeroInitialize_finalResult()
    {
        if (zeroInitializeSingleAxis)
        {
            String status;
            if (axes[axisToInitialize].initStatus == RobotConstants::InitStatus::ZEI_FINISHED)
            {
                status = "successful";
            }
            else if (axes[axisToInitialize].initStatus == RobotConstants::InitStatus::ZEI_FAILED)
            {
                status = "failed";
            }
            else
            {
                status = "ERROR IN CODE!";
            }

            String message = "Zero initialization for Axis " + String(axisToInitialize) + " " + status;
            addDataToOutQueue(message);

            axisToInitialize = 0;
            return;
        }

        String successfullAxes = "";
        String failedAxes = "";
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_ONGOING)
            {
                return; // Still ongoing for some axes
            }
            else if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_FINISHED)
            {
                successfullAxes += String(nodeId) + " ";
            }
            else if (axes[nodeId].initStatus == RobotConstants::InitStatus::ZEI_FAILED)
            {
                failedAxes += String(nodeId) + " ";
            }
        }

        String message = "Zero initialization process completed. Successful axes: " + successfullAxes + ". Failed axes: " + failedAxes;
        addDataToOutQueue(message);
    }

    void MoveControllerBase::regularHeartbeatCallback(uint8_t nodeId, uint8_t status)
    {
        String statusStr;
        if (status == 0x05)
        {
            statusStr = "operational";
        }
        else if (status == 0x04)
        {
            statusStr = "alarm";
        }
        else if (status == 0x7F)
        {
            statusStr = "pre-operational";
        }
        else if (status == 0x00)
        {
            statusStr = "boot-up";
        }
        else
        {
            statusStr = "unknown";
        }
        // addDataToOutQueue("Heartbeat " + String(node) + ": " + status);
        axes[nodeId].lastHeartbeatMs = millis();
    }

    void MoveControllerBase::checkTimeouts()
    {
        const uint32_t now = millis();
        for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
        {
            Axis& axis = axes[nodeId];
            const uint32_t lastHb = axis.lastHeartbeatMs;

            if ((now - lastHb) > RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS && axis.isAlive)
            {
                String message = "==== Heartbeat timeout for Axis " + String(nodeId) + " ====";
                axis.isAlive = false;
                //addDataToOutQueue(message);
            } else if ((now - lastHb) <= RobotConstants::Robot::HEARTBEAT_TIMEOUT_MS  && !axis.isAlive) {
                String message = "==== Heartbeat restored for Axis " + String(nodeId) + " ====";
                axis.isAlive = true;
                //addDataToOutQueue(message);
            }
        }
    }

    void MoveControllerBase::regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position)
    {
        if (!success)
        {
            String message = "Failed to read Position Actual Value for node " + String(nodeId);
            addDataToOutQueue(message);
            return;
        }

#ifdef DEBUG
        String message = "Position Actual Value from node " + String(nodeId) + ": " + String(position, HEX);
        addDataToOutQueue(message);
#endif

        positionUpdate(nodeId, position);
    }
}