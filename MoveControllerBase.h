#ifndef MOVECONTROLLERBASE_H

#define MOVECONTROLLERBASE_H

#include <string>
#include <unordered_map>
#include "CanOpen.h"
#include "Params.h"
#include "Axis.h"

namespace StepDirController
{

    class MoveControllerBase
    {
    public:
        void requestStatus(const std::vector<uint8_t> &nodeIds)
        {
            String reply = RobotConstants::Commands::MOTOR_STATUS + " " + RobotConstants::Status::OK + " ";
            for (uint8_t nodeId : nodeIds)
            {
                // Validate requested node ID to avoid accidental insertion and out-of-range access.
                if (nodeId == 0 || nodeId > axesCnt)
                {
                    reply += String(nodeId) + ":" + RobotConstants::Status::INVALID_NODE_ID + "; ";
                }
                else
                {
                    Axis &axis = axes.at(nodeId);
                    reply += String(nodeId) + ":" + String(axis.isAlive) + "," + String(axis.initStatus) + "; ";
                }
            }

            addDataToOutQueue(reply);
        }

        bool start(CanOpen *canOpen, uint8_t axesCnt);

        uint8_t getAxesCount() const { return axesCnt; }
        Axis &getAxis(uint8_t nodeId) { return axes.at(nodeId); }

        void setRegularSpeedUnits(double speed);        // настройка крейсерской скорости в единицах измерения в секунду (градусы в секунду)
        void setAccelerationUnits(double acceleration); // настройка ускорения в единицах измерения в секунду^2 (градусы в секунду^2)

        double getRegularSpeedUnits() const;
        double getAccelerationUnits() const;

        void startZeroInitializationAllAxes();
        void startZeroInitializationSingleAxis(uint8_t nodeId);

        void move();

        // Call this regularly from the main loop to check timeouts.
        void tick();

        // TODO: добавить метод, например, void tick(), вызывать его из loop в ino-файле
        // метод tick() должен вызывать метод чтения (который нужно написать) у CanOpen (который сам взаимодействует с шиной CAN)
        // CanOpen может возвращать значения через колбэки
        // полученные значения через колбэки сохранять в Axis

        // TODO: метод tick должен рассылать запросы по двигателям о их состоянии (напряжение, температура, т.д. - все что есть по протоколу)

        void tick_requestPosition(const std::vector<uint8_t> &nodeIds);

    protected:
        void prepareMove();

    private:
        CanOpen *canOpen;
        std::unordered_map<uint8_t, Axis> axes;

        bool initialized = false;
        uint8_t axesCnt = 0;

        bool zeroInitializeSingleAxis = true;
        uint8_t axisToInitialize = 0;

        double regularSpeedUnits = 1.0f; // The speed of the maximum moving axis in percent of full speed (1.0 = 100%)
        double accelerationUnits = 1.0f; // The acceleration of the maximum moving axis in percent of full acceleration (1.0 = 100%)

        void sendMove();

        void positionUpdate(uint8_t nodeId, int32_t position);

        void setRegularPositionActualValueCallback(uint8_t nodeId);

        void tick_checkTimeouts();
        void tick_checkZEITimeouts();

        bool checkResponseStatus(uint8_t nodeId, bool success, String errorMessage);
        // Callbacks
        void zeroInitialize_start(uint8_t nodeId);
        void zeroInitialize_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success);
        void zeroInitialize_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success);
        void zeroInitialize_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success);
        void zeroInitialize_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success);

        void zeroInitialize_finalResult();

        void regularHeartbeatCallback(uint8_t nodeId, uint8_t status);
        void regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position);
    };

}

#endif
