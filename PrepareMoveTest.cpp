#include <vector>
#include "PrepareMoveTest.h"
#include "Axis.h"
#include "Debug.h"

namespace
{
    struct PrepareMoveTestCase
    {
        const char *name;
        MoveController::MoveInput params;
        MoveController::PrepareMoveStatus expectedStatus;
        bool expectedSync;
    };

    MoveController::MoveInput makeParams(const std::array<double, RobotConstants::Robot::AXES_COUNT> &targetsDeg,
                                         double velocityPercent,
                                         double accelerationPercent)
    {
        MoveController::MoveInput input;
        for (uint8_t idx = 0; idx < RobotConstants::Robot::AXES_COUNT; ++idx)
        {
            input.relativeMotions[idx] = Axis::unitsToSteps(targetsDeg[idx]);
        }
        input.velocity = velocityPercent;
        input.acceleration = accelerationPercent;
        return input;
    }

    void emitPrepareMoveTestCaseResult(const MoveController::PrepareMoveComputationResult &result,
                                       bool verbose)
    {
        const String prefix = "PMT ";
        addDataToOutQueue(prefix + "status=" + MoveController::prepareMoveStatusToString(result.status));
        addDataToOutQueue(prefix + "reason=" + result.reason);
        addDataToOutQueue(prefix + "profile=" + String(result.isTriangularProfile ? "TRI" : "TRAP"));
        addDataToOutQueue(prefix + "ta=" + String(result.accelerationTimeSec, 6));
        addDataToOutQueue(prefix + "tc=" + String(result.constantVelocityTimeSec, 6));
        addDataToOutQueue(prefix + "tt=" + String(result.fullMovementTimeSec, 6));
        addDataToOutQueue(prefix + "max_axis=" + String(result.maxMovementAxisId));
        addDataToOutQueue(prefix + "max_steps=" + String(result.maxMovementAbsSteps));

        if (!verbose)
        {
            return;
        }

        addDataToOutQueue(prefix + "+ Detailed axis results: +");

        for (uint8_t nodeId = 1; nodeId <= RobotConstants::Robot::AXES_COUNT; ++nodeId)
        {
            const MoveController::PrepareMoveAxisResult &axisResult = result.axes[nodeId - 1];
            String axisLine =
                prefix +
                "axis=" + String(static_cast<char>(RobotConstants::Robot::MIN_NODE_ID + nodeId - 1)) +
                " req=" + String(axisResult.requestedMovement) +
                " target_steps=" + String(axisResult.targetSteps) +
                " vel_rpm=" + String(axisResult.profileVelocityRpm) +
                " acc_rpmps=" + String(axisResult.profileAccelerationRpmPerSec) +
                " vel_sps=" + String(axisResult.velocityStepsPerSec, 6) +
                " acc_sps2=" + String(axisResult.accelerationStepsPerSec2, 6);

            addDataToOutQueue(axisLine);
        }
    }
}

bool runPrepareMoveTests(bool verbose)
{

    const std::vector<PrepareMoveTestCase> testCases = {
        // {"nominal_trapezoid_1", makeParams({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 0.21, 0.12), MoveController::PrepareMoveStatus::OK, true},
        // {"nominal_trapezoid_2", makeParams({-12.12233432, 41.123992, -30.0000112, -40.0230, -50.00321, -60.1030}, 0.05, 0.3), MoveController::PrepareMoveStatus::OK, true},
        // {"mixed_distance", makeParams({0.05, 0.10, 120.0, -95.0, 0.2, 60.0}, 0.412, 0.6532), MoveController::PrepareMoveStatus::OK, true},
        // {"all_zero", makeParams({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1, 0.3), MoveController::PrepareMoveStatus::NO_EFFECTIVE_MOTION, true},
        // {"velocity_zero", makeParams({0.1, 0.3, 0.01, 0.03, 0.001, 0.003}, 0.0, 0.12), MoveController::PrepareMoveStatus::INVALID_SPEED, false},
        // {"acceleration_zero", makeParams({0.1, 0.3, 0.01, 0.03, 0.001, 0.003}, 0.1, 0.0), MoveController::PrepareMoveStatus::INVALID_ACCELERATION, false},
        // {"very_low_speed", makeParams({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 0.0001, 0.2), MoveController::PrepareMoveStatus::OK, true},
        // {"very_low_acceleration", makeParams({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 0.1, 0.000015259), MoveController::PrepareMoveStatus::OK, true},
        // {"low_speed_quantized", makeParams({10.0, 0.0000219726, 30.0, 0.00012726, 50.0, 60.0}, 0.0003331, 0.2), MoveController::PrepareMoveStatus::OK, true},
        // {"low_acceleration_quantized", makeParams({10.0, 0.3321, 0.00001, 40.0, 1.0, 60.0}, 0.1, 0.000013), MoveController::PrepareMoveStatus::OK, true},
        // {"low_speed_and_acceleration_quantized", makeParams({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 0.00031, 0.000001), MoveController::PrepareMoveStatus::OK, true}, // All velocities and acceleration will be 1 and 1
        // {"tiny_angle_and_low_velocity", makeParams({0.000219, 0.000002, 0.0003, 0.000004, 0.01, 0.000006}, 0.000354, 0.2), MoveController::PrepareMoveStatus::OK, true},
        // {"tiny_angle_and_low_acceleration", makeParams({0.000219, 0.02219, 0.0003, 0.000004, 0.01, 0.000006}, 0.1, 0.000016), MoveController::PrepareMoveStatus::OK, true},
        // {"tiny_angle_and_low_velocity_and_acceleration", makeParams({0.000219, 0.000002, 0.0003, 0.000004, 0.01, 0.000006}, 0.000334, 0.00153), MoveController::PrepareMoveStatus::OK, true},
        // {"mixed_angles", makeParams({0.000219, 0.02219, 32.32, -95.0, 0.01, 0.000006}, 0.1, 0.02153), MoveController::PrepareMoveStatus::OK, true},
        // {"mixed_angles_low_speed", makeParams({23.1, 0.02219, 1.0, 5.0, 0.01, 0.000006}, 0.000334, 0.02153), MoveController::PrepareMoveStatus::OK, true},
        // {"mixed_angles_low_acceleration", makeParams({0.000219, 0.02219, 120.0, -95.0, 0.01, 0.000006}, 0.1, 0.00153), MoveController::PrepareMoveStatus::OK, true},
        // {"mixed_angles_low_speed_and_acceleration", makeParams({23.1, 0.00219, 1.0, 5.0, 0.01, 0.000006}, 0.000334, 0.00153), MoveController::PrepareMoveStatus::OK, true},
        // {"tiny_quantized", makeParams({0.000001, -0.000001, 0.0, 0.0, 0.0, 0.0}, 0.05, 0.15), MoveController::PrepareMoveStatus::NO_EFFECTIVE_MOTION, true},
        //{"normal_1", makeParams({9.0, 0.0, -20.0, 0.0, 10.0, 0.0}, 0.05, 0.1), MoveController::PrepareMoveStatus::OK, true}
        {"normal_1", makeParams({0.0, -1.0, 0.01, 0.003, -1.033421}, 0.1, 0.032), MoveController::PrepareMoveStatus::OK, true}};

    uint32_t passed = 0;
    uint32_t caseId = 0;
    for (const PrepareMoveTestCase &testCase : testCases)
    {
        ++caseId;
        String caseStartLine1 = "\n======= Case (" + String(caseId) + ") " + String(testCase.name) + " =======\n\n++ INPUTS ++\n";

        String caseStartLine2 = "Velocity (percent): " + String(testCase.params.velocity, 5) + ", Acceleration (percent): " + String(testCase.params.acceleration, 5);
        String movementValues;
        movementValues.reserve(100);
        for (uint8_t axisIdx = 0; axisIdx < RobotConstants::Robot::AXES_COUNT; ++axisIdx)
        {
            movementValues += "J" + String(static_cast<char>(RobotConstants::Robot::MIN_NODE_ID + axisIdx)) + String(testCase.params.relativeMotions[axisIdx]) + " ";
        }

        addDataToOutQueue(caseStartLine1);
        addDataToOutQueue(caseStartLine2);
        addDataToOutQueue(movementValues);

        MoveController::MoveInput inputCopy = testCase.params; // computePrepareMove can modify the input (e.g. quantize very low speeds), so we pass a copy to preserve the original values for logging
        MoveController::PrepareMoveComputationResult result = MoveController::computePrepareMove(inputCopy);

        const bool statusPass = result.status == testCase.expectedStatus;
        if (statusPass)
        {
            ++passed;
        }
        String verdict = "\n++ OUTPUTS ++\n\nstatusExpected=" + MoveController::prepareMoveStatusToString(testCase.expectedStatus) +
                         "; statusGot=" + MoveController::prepareMoveStatusToString(result.status) +
                         "\nsyncExpected=" + String(testCase.expectedSync);

        addDataToOutQueue(verdict);
        emitPrepareMoveTestCaseResult(result, verbose);
    }

    addDataToOutQueue("\n======= Test summary =======");
    addDataToOutQueue(RobotConstants::Commands::PREPAREMOVE_TEST + " SUMMARY total=" + String(testCases.size()) + " passed=" + String(passed) + " failed=" + String(testCases.size() - passed));
    return passed == testCases.size();
}