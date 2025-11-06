#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H
#include "Axis.h"

constexpr int MaxAxes = 10;
namespace StepDirController{
class ControllerBase{
    public:
        bool isMoving() const { return movingInProgress; } // Возвращает TRUE, если с овершается перемещение
        virtual void setRegularSpeedUnits(double speed) { regularSpeedUnits = speed; } // настройка крейсерской скорости в единицах измерения в секунду
        virtual void setAccelerationUnits(double acceleration) { accelerationUnits = acceleration; } // настройка ускорения в единицах измерения в секунду^2
        int getAxesCount() { return axesCnt; }

    protected:
        ControllerBase() : axisCurrent(0) { }

        bool movingInProgress = false;
        Axis* axisList[MaxAxes + 1]; // список указателей на оси
        int axesCnt = 0;
        Axis* leadAxis;

        uint32_t startGlobalTime;
        uint32_t stepsAll;
        volatile uint32_t stepsDoneAll;

        uint8_t axisCurrent;

        double regularSpeedUnits = 1.0f; // скорость в единицах измерения в секунду
        double accelerationUnits = 1.0f; // ускорение в единицах измерения в секунду^2

        void (*onStopMove)() = nullptr; // callback после завершения движения

        template<typename... Axes>
        void attachAxes(Axis& axis, Axes&... axes);

        void attachAxes(Axis& axis){
            axisList[axisCurrent++] = &axis;
            axesCnt++;
            axisList[axisCurrent] = nullptr;
            axisCurrent = 0;
        }
};

template<typename... Axes>
void ControllerBase::attachAxes(Axis& axis, Axes&... axes)
{
    static_assert(sizeof...(axes) < MaxAxes, "Too many axes used. Please increase MaxAxes in file ControllerBase.h");

    axisList[axisCurrent++] = &axis;
    axesCnt++;
    attachAxes(axes...);
}
}

#endif