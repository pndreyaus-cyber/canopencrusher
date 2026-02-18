#include <vector>
#include "PrepareMoveMathTestRunner.h"
#include "Axis.h"
#include "Debug.h"

namespace StepDirController
{
    namespace
    {
        struct PrepareMoveMathTestCase
        {
            const char *name;
            MoveParams<RobotConstants::Robot::AXES_COUNT> params;
            MoveControllerBase::PrepareMoveStatus expectedStatus;
            bool expectedSync;
            bool expectedQuantized;
        };

        MoveParams<RobotConstants::Robot::AXES_COUNT> makeParams(const std::array<double, RobotConstants::Robot::AXES_COUNT> &targetsDeg,
                                                                  double speedDegPerSec,
                                                                  double accelerationDegPerSec2)
        {
            MoveParams<RobotConstants::Robot::AXES_COUNT> params;
            for (uint8_t idx = 0; idx < RobotConstants::Robot::AXES_COUNT; ++idx)
            {
                params.movementUnits[idx] = targetsDeg[idx];
            }
            params.speed = Axis::speedUnitsToMotorRPM(speedDegPerSec);
            params.acceleration = Axis::accelerationUnitsToRPMPS(accelerationDegPerSec2);
            return params;
        }

        void emitPrepareMoveTestCaseResult(uint32_t caseId,
                                           const String &caseName,
                                           const MoveParams<RobotConstants::Robot::AXES_COUNT> &params,
                                           const MoveControllerBase::PrepareMoveComputationResult &result,
                                           bool pass,
                                           const String &verdictReason,
                                           bool verbose)
        {
            String line = RobotConstants::Commands::PREPAREMOVE_MATH_TEST + " RESULT"
                          + " case=" + String(caseId)
                          + " name=" + caseName
                          + " pass=" + String(pass)
                          + " status=" + MoveControllerBase::prepareMoveStatusToString(result.status)
                          + " reason=" + result.reason
                          + " profile=" + String(result.isTriangularProfile ? "TRI" : "TRAP")
                          + " ta=" + String(result.accelerationTimeSec, 6)
                          + " tc=" + String(result.constantVelocityTimeSec, 6)
                          + " tt=" + String(result.fullMovementTimeSec, 6)
                          + " sync=" + String(result.syncModelValid)
                          + " quantized=" + String(result.hasQuantizedToZero)
                          + " max_axis=" + String(result.maxMovementAxisId)
                          + " max_steps=" + String(result.maxMovementAbsSteps)
                          + " speed_rpm=" + String(params.speed)
                          + " accel_rpmps=" + String(params.acceleration)
                          + " verdict=" + verdictReason;
            addDataToOutQueue(line);

            if (!verbose)
            {
                return;
            }

            for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
            {
                const MoveControllerBase::PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];
                String axisLine = RobotConstants::Commands::PREPAREMOVE_MATH_TEST + " AXIS"
                                  + " case=" + String(caseId)
                                  + " axis=" + String(static_cast<char>(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1))
                                  + " req=" + String(axisResult.requestedMovement)
                                  + " qz=" + String(axisResult.quantizedToZero)
                                  + " target_steps=" + String(axisResult.targetSteps)
                                  + " rel_steps=" + String(axisResult.relativeSteps)
                                  + " vel_rpm=" + String(axisResult.profileVelocityRpm)
                                  + " acc_rpmps=" + String(axisResult.profileAccelerationRpmPerSec)
                                  + " vel_sps=" + String(axisResult.velocityStepsPerSec, 6)
                                  + " acc_sps2=" + String(axisResult.accelerationStepsPerSec2, 6);
                addDataToOutQueue(axisLine);
            }
        }
    }

    bool runPrepareMoveMathTestSuite(MoveControllerBase &controller, bool verbose)
    {
        if (!controller.isInitialized())
        {
            addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_MATH_TEST + " " + RobotConstants::Status::LOGIC_ERROR + " reason=NOT_INITIALIZED");
            return false;
        }

        if (controller.isMoveInProgress())
        {
            addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_MATH_TEST + " " + RobotConstants::Status::LOGIC_ERROR + " reason=CONTROLLER_BUSY");
            return false;
        }

        const std::vector<PrepareMoveMathTestCase> testCases = {
            {"nominal_trapezoid", makeParams({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 120.0, 2000.0), MoveControllerBase::PrepareMoveStatus::OK, true, false},
            {"mixed_distance", makeParams({0.05, 0.10, 120.0, -95.0, 0.2, 60.0}, 140.0, 2200.0), MoveControllerBase::PrepareMoveStatus::OK, true, false},
            {"all_zero", makeParams({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 120.0, 2000.0), MoveControllerBase::PrepareMoveStatus::NO_EFFECTIVE_MOTION, true, false},
            {"tiny_quantized", makeParams({0.000001, -0.000001, 0.0, 0.0, 0.0, 0.0}, 120.0, 2000.0), MoveControllerBase::PrepareMoveStatus::NO_EFFECTIVE_MOTION, true, true},
            {"boundary_speed_zero", makeParams({10.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0, 2000.0), MoveControllerBase::PrepareMoveStatus::INVALID_SPEED, false, false},
            {"boundary_acc_zero", makeParams({10.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 120.0, 0.0), MoveControllerBase::PrepareMoveStatus::INVALID_ACCELERATION, false, false},
        };

        uint32_t passed = 0;
        uint32_t caseId = 0;
        for (const PrepareMoveMathTestCase &testCase : testCases)
        {
            ++caseId;
            String startLine = RobotConstants::Commands::PREPAREMOVE_MATH_TEST + " START"
                               + " case=" + String(caseId)
                               + " name=" + String(testCase.name)
                               + " speed_rpm=" + String(testCase.params.speed)
                               + " accel_rpmps=" + String(testCase.params.acceleration);
            for (uint8_t axisIdx = 0; axisIdx < RobotConstants::Robot::AXES_COUNT; ++axisIdx)
            {
                startLine += " J" + String(static_cast<char>(RobotConstants::Robot::MIN_NODE_ID + axisIdx)) + String(testCase.params.movementUnits[axisIdx], 6);
            }
            addDataToOutQueue(startLine);

            MoveControllerBase::PrepareMoveComputationResult result = controller.computePrepareMoveForTesting(testCase.params);

            const bool statusPass = result.status == testCase.expectedStatus;
            const bool syncPass = (testCase.expectedSync == result.syncModelValid);
            const bool quantPass = (testCase.expectedQuantized == result.hasQuantizedToZero);
            const bool pass = statusPass && syncPass && quantPass;
            if (pass)
            {
                ++passed;
            }

            String verdictReason = "statusExpected=" + MoveControllerBase::prepareMoveStatusToString(testCase.expectedStatus)
                                   + ",statusGot=" + MoveControllerBase::prepareMoveStatusToString(result.status)
                                   + ",syncExpected=" + String(testCase.expectedSync)
                                   + ",syncGot=" + String(result.syncModelValid)
                                   + ",quantExpected=" + String(testCase.expectedQuantized)
                                   + ",quantGot=" + String(result.hasQuantizedToZero);
            emitPrepareMoveTestCaseResult(caseId, String(testCase.name), testCase.params, result, pass, verdictReason, verbose);
        }

        addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_MATH_TEST + " SUMMARY total=" + String(testCases.size()) + " passed=" + String(passed) + " failed=" + String(testCases.size() - passed));
        return passed == testCases.size();
    }
}
