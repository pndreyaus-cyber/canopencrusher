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
        void requestStatus();
        int32_t axisPosition(uint8_t nodeId) { return axes.at(nodeId).getCurrentPositionInSteps(); }

        bool start(CanOpen *canOpen, uint8_t axesCnt);

        uint8_t getAxesCount() const { return axesCnt; }
        Axis &getAxis(uint8_t nodeId) { return axes.at(nodeId); }

        void setRegularSpeedUnits(double speed);        // настройка крейсерской скорости в единицах измерения в секунду (градусы в секунду)
        void setAccelerationUnits(double acceleration); // настройка ускорения в единицах измерения в секунду^2 (градусы в секунду^2)

        // double getRegularSpeedUnits() const;
        // double getAccelerationUnits() const;

        void startZeroInitializationAllAxes();
        void startZeroInitializationSingleAxis(uint8_t nodeId);

        void move();

        // Call this regularly from the main loop to check timeouts.
        void tick_50();
        void tick_500();


    protected:
        void prepareMove();

    private:
        CanOpen *canOpen;
        std::unordered_map<uint8_t, Axis> axes;
        uint8_t axesCnt = 0;
        bool initialized = false;

        double regularSpeedUnits = 1.0f; // The speed of the maximum moving axis in percent of full speed (1.0 = 100%)
        double accelerationUnits = 1.0f; // The acceleration of the maximum moving axis in percent of full acceleration (1.0 = 100%)

        void sendMove();

        void positionUpdate(uint8_t nodeId, int32_t position);

        // Helper, so that not to write the long time every time
        void setRegularPositionActualValueCallback(uint8_t nodeId);

        // ======== Timer functions ========
        void tick_checkTimeouts();
        void tick_checkZEITimeouts();
        void tick_requestPosition();
        // ======== Timer functions end ========

        // ======== ZEI Sequence ======== 
        bool zeroInitializeSingleAxis = true;
        uint8_t axisToInitialize = 0;

        void ZEI_start(uint8_t nodeId);
        void ZEI_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success);
        void ZEI_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success);
        void ZEI_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success);
        void ZEI_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success);
        void ZEI_finalResult();

        bool ZEI_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage);
        // ======== ZEI Sequence End ========

        // ======== Regular callbacks ========
        void regularHeartbeatCallback(uint8_t nodeId, uint8_t status);
        void regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position);
        // ======== Regular callbacks end ========
    };

}

#endif
