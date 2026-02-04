#include <math.h>
#include <iostream>
#include "Arduino.h"
#include "Axis.h"
#include "RobotConstants.h"

extern void addDataToOutQueue(String data);

namespace StepDirController
{

    Axis::Axis() : nodeId(kInvalidNodeId) {
        initialized = false;
        initStatus = RobotConstants::InitStatus::ZEI_NONE;
    }

    Axis::Axis(uint8_t nodeId) : nodeId(nodeId)
    {
        init_od_ram(&params);
        movementUnits = 0.0;
        params.x6064_positionActualValue = 0;
        stepsPerRevolution = RobotConstants::Axis::DEFAULT_STEPS_PER_REVOLUTION;
        unitsPerRevolution = RobotConstants::Axis::DEFAULT_UNITS_PER_REVOLUTION;
        initStatus = RobotConstants::InitStatus::ZEI_NONE;
        initialized = true;
    }

    Axis &Axis::setStepsPerRevolution(uint32_t steps)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setStepsPerRevolution -- Axis not initialized");
#endif
            return *this;
        }
        stepsPerRevolution = steps;
        return *this;
    }

    Axis &Axis::setUnitsPerRevolution(double units)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setUnitsPerRevolution -- Axis not initialized");
#endif
            return *this;
        }
        unitsPerRevolution = units;
        return *this;
    }

    Axis &Axis::setCurrentPositionInUnits(double units)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setCurrentPositionInUnits -- Axis not initialized");
#endif
            return *this;
        }

        int32_t steps = std::round(unitsToSteps(units)); // Edited for C++
        return setCurrentPositionInSteps(steps);
    }

    Axis &Axis::setCurrentPositionInSteps(int32_t steps)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setCurrentPositionInSteps -- Axis not initialized");
#endif
            return *this;
        }

        params.x6064_positionActualValue = steps;
        return *this;
    }

    int32_t Axis::getCurrentPositionInSteps() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getCurrentPositionInSteps -- Axis not initialized");
#endif
            return -1;
        }

        return params.x6064_positionActualValue;
    }

    bool Axis::setTargetPositionRelativeInUnits(double units)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setTargetPositionRelativeInUnits -- Axis not initialized");
#endif
            return false;
        }

        int32_t steps = std::round(unitsToSteps(units)); // Edited for C++

        return setTargetPositionRelativeInSteps(steps);
    }

    bool Axis::setTargetPositionRelativeInSteps(int32_t steps)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setTargetPositionRelativeInSteps -- Axis not initialized");
#endif
            return false;
        }
        return setTargetPositionAbsoluteInSteps(steps + params.x6064_positionActualValue); // Проверить
    }

    bool Axis::setTargetPositionAbsoluteInUnits(double units)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setTargetPositionAbsoluteInUnits -- Axis not initialized");
#endif
            return false;
        }

        int32_t steps = std::round(unitsToSteps(units)); // Edited for C++
        return setTargetPositionAbsoluteInSteps(steps);
    }

    bool Axis::setTargetPositionAbsoluteInSteps(int32_t steps)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::setTargetPositionAbsoluteInSteps -- Axis not initialized");
#endif
            return false;
        }

        int32_t relativePosition = steps - getCurrentPositionInSteps();
        movementUnits = stepsToUnits(relativePosition);
        movementSteps = std::fabs(relativePosition);
        params.x607A_targetPosition = steps;
        return true;
    }

    double Axis::getPositionInUnits() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getPositionInUnits -- Axis not initialized");
#endif
            return -1;
        }

        double position = stepsToUnits(getCurrentPositionInSteps());
        return position;
    }

    uint32_t Axis::getStepsPerRevolution() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getStepsPerRevolution -- Axis not initialized");
#endif
            return 0;
        }

        return stepsPerRevolution;
    }

    double Axis::getUnitsPerRevolution() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getUnitsPerRevolution -- Axis not initialized");
#endif
            return -1;
        }

        return unitsPerRevolution;
    }

    double Axis::stepsToUnits(int32_t steps) const // Перевести шаги в градусы
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::stepsToUnits -- Axis not initialized");
#endif
            return -1;
        }

        if (stepsPerRevolution == 0 || unitsPerRevolution == 0)
        { // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
            addDataToOutQueue("Axis::stepsToUnits -- division by zero");
            return 0;
        }
        return steps / (double)stepsPerRevolution * unitsPerRevolution;
    }

    int32_t Axis::unitsToSteps(double units) const // Перевести градусы в шаги
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::unitsToSteps -- Axis not initialized");
#endif
            return 0;
        }

        if (stepsPerRevolution == 0 || unitsPerRevolution == 0)
        { // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
            addDataToOutQueue("Axis::unitsToSteps -- division by zero");
            return 0;
        }
        return units / unitsPerRevolution * stepsPerRevolution;
    }

    uint32_t Axis::speedUnitsToRevolutionsPerMinute(double speedUnits) const // Перевести градусы/сек в об/мин
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::speedUnitsToRevolutionsPerMinute -- Axis not initialized");
#endif
            return 0;
        }

        if (unitsPerRevolution == 0)
        { // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
            addDataToOutQueue("Axis::speedUnitsToRevolutionsPerMinute -- division by zero");
            return 0;
        }
        return speedUnits * RobotConstants::Math::SECONDS_IN_MINUTE / unitsPerRevolution;
    }

    double Axis::revolutionsPerMinuteToSpeedUnits(uint32_t rpm) const // Перевести из об/мин в градусы/сек
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::revolutionsPerMinuteToSpeedUnits -- Axis not initialized");
#endif
            return 0;
        }
        return (double)rpm / RobotConstants::Math::SECONDS_IN_MINUTE * unitsPerRevolution;
    }

    uint32_t Axis::accelerationUnitsTorpmPerSecond(double accelearionUnits) const // Перевести градусы/сек^2 в об/(мин*сек)
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::accelerationUnitsTorpmPerSecond -- Axis not initialized");
#endif
            return 0;
        }
        if (unitsPerRevolution == 0)
        { // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
            addDataToOutQueue("Axis::accelerationUnitsTorpmPerSecond -- division by zero");
            return 0;
        }
        return accelearionUnits * RobotConstants::Math::SECONDS_IN_MINUTE / unitsPerRevolution;
    }

    double Axis::rpmPerSecondToAccelerationUnits(double rpmPerSecond) const // Перевести об/(мин*сек) в градусы/сек^2
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::rpmPerSecondToAccelerationUnits -- Axis not initialized");
#endif
            return 0;
        }

        if (unitsPerRevolution == 0)
        { // TODO: вместо условия выводить ошибку при компиляции, например static_assert(stepsPerRevolution == 0, "Not defined: stepsPerRevolution.. ");
            addDataToOutQueue("Axis::rpmPerSecondToAccelerationUnits -- division by zero");
            return 0;
        }
        return (double)rpmPerSecond / RobotConstants::Math::SECONDS_IN_MINUTE * unitsPerRevolution; // TODO: потенциальная потеря точности расчетов из-за SECONDS_IN_MINUTE, лучше объявить как константу float/double
    }

    uint8_t Axis::getNodeId() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getNodeId -- Axis not initialized");
#endif
            return 0;
        }

        return nodeId;
    }

    double Axis::getMovementUnits() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getMovementUnits -- Axis not initialized");
#endif
            return -1;
        }
        return movementUnits;
    }

    int32_t Axis::getTargetPositionAbsolute() const
    {
        if (!initialized)
        {
#ifdef DEBUG
            addDataToOutQueue("Axis::getTargetPositionAbsolute -- Axis not initialized");
#endif
            return -1;
        }

        return params.x607A_targetPosition;
    }

}