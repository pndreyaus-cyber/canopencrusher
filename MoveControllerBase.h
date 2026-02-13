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

        void startZeroInitializationAllAxes();
        void startZeroInitializationSingleAxis(uint8_t nodeId);

        bool move(MoveParams<RobotConstants::Robot::AXES_COUNT> params);

        // Call this regularly from the main loop to check timeouts.
        void tick_100();
        void tick_500();

    protected:
        bool prepareMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params);

        // void prepareMove(MoveParams<RobotConstants::Robot::AXES_COUNT> params);
    private:
        CanOpen *canOpen;
        std::unordered_map<uint8_t, Axis> axes;
        uint8_t axesCnt = 0;
        bool initialized = false;

        void positionUpdate(uint8_t nodeId, int32_t position);

        // Helper, so that not to write the long time every time
        void setRegularPositionActualValueCallback(uint8_t nodeId);

        // ======== Timer functions ========
        void tick_checkTimeouts();
        void tick_checkZEITimeouts();
        void tick_requestPosition();
        void tick_checkMAJStatusWord();
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

        // ======== MAJ Sequence ========
        void MAJ_start(uint8_t nodeId);
        void MAJ_afterWriteTo_0x6081(uint8_t nodeId, bool success);
        void MAJ_afterWriteTo_0x6040(uint8_t nodeId, bool success);
        void MAJ_afterWriteTo_0x6083(uint8_t nodeId, bool success);
        void MAJ_TPDO1(uint8_t nodeId, int32_t actualLocation, uint16_t statusWord);
        void MAJ_statusWordCallback(uint8_t nodeId, bool success, uint16_t statusWord);
        void MAJ_finalResult();

        bool MAJ_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage);
        bool MAJ_checkTargetPositionReached(uint16_t statusWord);
        // ======== MAJ Sequence End ========

        // ======== Regular callbacks ========
        void regularHeartbeatCallback(uint8_t nodeId, uint8_t status);
        void regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position);
        // ======== Regular callbacks end ========
    };

}

#endif
