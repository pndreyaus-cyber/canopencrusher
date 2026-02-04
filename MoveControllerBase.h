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

        void printStatus() {
            addDataToOutQueue("MoveControllerBase Status: " + String(axes.size()) + " axes configured. " + axesCnt + " axes count."); 
            for(const auto& [nodeId, axis] : axes) {
                String status = "Axis " + String(nodeId) + ": " + String(axis.isAlive) + ", Last Heartbeat: " + String(axis.lastHeartbeatMs) + " ms " + String(RobotConstants::initStatusToString(axis.initStatus));
                addDataToOutQueue(status);
            }
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
        void setWorkMode(uint8_t nodeId, uint8_t mode);
        void setControlWord(uint8_t nodeId, uint16_t controlWord);

        void move();

        // Call this regularly from the main loop to check timeouts.
        void tick();

        // TODO: добавить метод, например, void tick(), вызывать его из loop в ino-файле
        // метод tick() должен вызывать метод чтения (который нужно написать) у CanOpen (который сам взаимодействует с шиной CAN)
        // CanOpen может возвращать значения через колбэки
        // полученные значения через колбэки сохранять в Axis

        // TODO: метод tick должен рассылать запросы по двигателям о их состоянии (напряжение, температура, т.д. - все что есть по протоколу)

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

        void checkTimeouts();

        bool checkResponseStatus(uint8_t nodeId, bool success, String errorMessage);
        // Callbacks
        void zeroInitialize_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success);
        void zeroInitialize_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success);
        void zeroInitialize_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success);
        void zeroInitialize_AfterWriteTo_0x6060(uint8_t nodeId, bool success);
        void zeroInitialize_AfterReadPosition_0x6064(uint8_t nodeId, bool success, int32_t position);
        void zeroInitialize_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success);
        void zeroInitialize_AfterWriteTo_0x607A(uint8_t nodeId, bool success);
        void zeroInitialize_AfterReadStatusword_0x6041(uint8_t nodeId, bool success, uint16_t statusWord);

        void zeroInitialize_finalResult();

        void regularHeartbeatCallback(uint8_t nodeId, uint8_t status);
        void regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position);
    };

}

#endif
