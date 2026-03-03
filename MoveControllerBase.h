#ifndef MOVECONTROLLERBASE_H

#define MOVECONTROLLERBASE_H

#include <array>
#include <string>
#include <unordered_map>
#include <optional>

#include "CanOpen.h"
#include "Params.h"
#include "Axis.h"

namespace StepDirController
{

    class MoveControllerBase
    {
    public:
        enum class PrepareMoveStatus : uint8_t
        {
            OK = 0,
            NO_EFFECTIVE_MOTION = 1,
            INVALID_SPEED = 2,
            INVALID_ACCELERATION = 3,
            INVALID_PROFILE = 4,
            INVALID_PROFILE_OUT_OF_LIMITS = 5,
            INVALID_TIMING = 6,
            NOT_INITIALIZED = 7,
            OTHER_COMMAND_IN_PROGRESS = 8,
        };

        struct PrepareMoveAxisResult
        {
            bool requestedMovement = false;
            int32_t targetSteps = 0;
            uint32_t profileVelocityRpm = 0;
            uint32_t profileAccelerationRpmPerSec = 0;
            double velocityStepsPerSec = 0.0;
            double accelerationStepsPerSec2 = 0.0;
        };

        struct PrepareMoveComputationResult
        {
            PrepareMoveStatus status = PrepareMoveStatus::NO_EFFECTIVE_MOTION;
            String reason;
            bool isTriangularProfile = false;
            uint8_t maxMovementAxisId = 0;
            int32_t maxMovementAbsSteps = 0;
            double accelerationTimeSec = 0.0;
            double constantVelocityTimeSec = 0.0;
            double fullMovementTimeSec = 0.0;
            std::array<PrepareMoveAxisResult, RobotConstants::Robot::AXES_COUNT> axes;
        };

        struct MoveInput
        {
            double velocity;
            double acceleration;
            int32_t relativeMotions[RobotConstants::Robot::AXES_COUNT];
        };

        void requestStatus();
        std::optional<int32_t> axisPosition(uint8_t nodeId);

        ParamsStatusStruct start(CanOpen *canOpen, uint8_t axesCnt, bool writeNewLimitsToEEPROM = false, uint8_t* nodesToInvert = nullptr, uint8_t nodesToInvertCnt = 0);

        uint8_t getAxesCount() const { return axesCnt; }
        Axis &getAxis(uint8_t nodeId) { return axes.at(nodeId); }

        void startZeroInitializationAllAxes();
        void startZeroInitializationSingleAxis(uint8_t nodeId);

        PrepareMoveStatus move(MoveParams<RobotConstants::Robot::AXES_COUNT> params, bool isAbsoluteMove, const String *commandNameForLogging = nullptr);

        bool isInitialized() const { return initialized; }

        // Call this regularly from the main loop to check timeouts.
        void tick_100();
        void tick_500();

        static PrepareMoveComputationResult computePrepareMove(MoveInput &input);
        static String prepareMoveStatusToString(PrepareMoveStatus status);

        bool isMoveInProgress() const { return isMAJInProgress; }

    protected:
        PrepareMoveComputationResult prepareMove(const MoveParams<RobotConstants::Robot::AXES_COUNT> &params, bool isAbsoluteMove);

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
        void ZEI_AfterWriteTo_0x6081(uint8_t nodeId, bool success);
        void ZEI_AfterFirstWriteTo_0x6040(uint8_t nodeId, bool success);
        void ZEI_AfterFirstWriteTo_0x260A(uint8_t nodeId, bool success);
        void ZEI_AfterSecondWriteTo_0x260A(uint8_t nodeId, bool success);
        void ZEI_AfterSecondWriteTo_0x6040(uint8_t nodeId, bool success);
        void ZEI_finalResult();

        bool ZEI_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage);
        // ======== ZEI Sequence End ========

        // ======== MAJ Sequence ========
        bool isMAJInProgress = false;
        const String *moveCommandName = nullptr; // For logging purposes, to know which command triggered the MAJ

        void MAJ_start(uint8_t nodeId);
        void MAJ_afterRequestOf_0x6040(uint8_t nodeId, bool success, uint16_t controlWord);
        void MAJ_afterWriteTo_0x6040(uint8_t nodeId, bool success);
        void MAJ_setTargetVelocity(uint8_t nodeId);
        void MAJ_afterWriteTo_0x6081(uint8_t nodeId, bool success);
        void MAJ_afterWriteTo_0x6083(uint8_t nodeId, bool success);
        void MAJ_TPDO4(uint8_t nodeId, int32_t actualLocation, uint16_t statusWord);
        void MAJ_SYNCFunnel();
        void MAJ_requestStatusWord(uint8_t nodeId);
        void MAJ_statusWordCallback(uint8_t nodeId, bool success, uint16_t statusWord);
        void MAJ_finalResult();

        bool MAJ_checkResponseStatus(uint8_t nodeId, bool success, String errorMessage);
        bool MAJ_checkTargetPositionReached(uint16_t statusWord);

        void MAJ_clearMoveStatusesAfterMoveCompletion()
        {
            for (uint8_t nodeId = 1; nodeId <= axesCnt; ++nodeId)
            {
                axes[nodeId].moveStatus = RobotConstants::MoveStatus::NOT_TASKED_WITH_MOVE;
            }
        }
        // ======== MAJ Sequence End ========

        // ======== Fixate Axis after restoring life ========
        void FAL_start(uint8_t nodeId);
        void FAL_afterWriteTo_0x6040(uint8_t nodeId, bool success);

        // ======== End of Fixate Axis after restoring life ========

        // ======== Regular callbacks ========
        void regularHeartbeatCallback(uint8_t nodeId, uint8_t status);
        void regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position);
        // ======== Regular callbacks end ========
    };

}

#endif
