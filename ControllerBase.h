#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H
#include "Axis.h"

constexpr int MaxAxes = 10;
namespace StepDirController{
class ControllerBase{
    public:
        bool isMoving() const { return movingInProgress; } // Возвращает TRUE, если с овершается перемещение

        // void setOnStopMove(void (*callback)()) { this->onStopMove = callback; } // настройка уведомления о завершении перемещения

        //virtual void setJerkSpeedUnits(double jerk) { jerkSpeedUnits = jerk; } // настройка начальной/конечной скорости (рывка) в единицах измерения в секунду
        virtual void setRegularSpeedUnits(double speed) { regularSpeedUnits = speed; } // настройка крейсерской скорости в единицах измерения в секунду
        virtual void setAccelerationUnits(double acceleration) { accelerationUnits = acceleration; } // настройка ускорения в единицах измерения в секунду^2
        //virtual void setDecelerationUnits(double deceleration) { decelerationUnits = deceleration; } // настройка замедления в единицах измерения в секунду^2

        //virtual void stopAsync() = 0;
        //virtual void stop() = 0;

        //void emergencyStop() { noInterrupts(); referenceGenerator.end(); movingInProgress = false; interrupts();}
        int getAxesCount() { return axesCnt; }

    protected:
        ControllerBase() : axisCurrent(0) { }

        //uint32_t period; // длительность периода генератора в микросекундах

        //IntervalTimer referenceGenerator; // генератор прерываний для движения

        bool movingInProgress = false;

        Axis* axisList[MaxAxes + 1]; // список указателей на оси
        int axesCnt = 0;
        Axis* leadAxis;

        uint32_t startGlobalTime;
        uint32_t stepsAll;
        volatile uint32_t stepsDoneAll;

        uint8_t axisCurrent;

        //double jerkSpeedUnits = 1.0f; // рывок в единицах измерения в секунду
        double regularSpeedUnits = 1.0f; // скорость в единицах измерения в секунду
        double accelerationUnits = 1.0f; // ускорение в единицах измерения в секунду^2
        //double decelerationUnits = -1.0f; // замедление в единицах измерения в секунду^2

        void (*onStopMove)() = nullptr; // callback после завершения движения

        template<typename... Axes>
        void attachAxes(Axis& axis, Axes&... axes);

        void attachAxes(Axis& axis){
            axisList[axisCurrent++] = &axis;
            axesCnt++;
            axisList[axisCurrent] = nullptr;
            axisCurrent = 0;
        }

        //virtual void tick() = 0; // функия тактирования двигателей, вызываемая генератором
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