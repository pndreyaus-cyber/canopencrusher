#ifndef MOVECONTROLLERBASE_H
#define MOVECONTROLLERBASE_H

#include "ControllerBase.h"
#include "MyCanOpen.h"
#include "Params.h"
#include <string>

namespace StepDirController{ 
class MoveControllerBase : public ControllerBase{
    public:
        //MoveControllerBase();
        MoveControllerBase(MyCanOpen *canOpen);
        
        //void setJerkSpeedUnits(double jerk) override; // настройка начальной/конечной скорости (рывка) в единицах измерения в секунду
        void setRegularSpeedUnits(double speed) override; // настройка крейсерской скорости в единицах измерения в секунду (градусы в секунду)
        void setAccelerationUnits(double acceleration) override; // настройка ускорения в единицах измерения в секунду^2 (градусы в секунду^2)

        double getRegularSpeedUnits() const;
        double getAccelerationUnits() const;
        //void setDecelerationUnits(double deceleration) override; // настройка замедления в единицах измерения в секунду^2

        void setOnMoveStarted(void (*callback)());
        void setOnMoveFinished(void (*callback)());
        void setOnEmergensyStoped(void (*callback)());


        template<typename... Axes>
        ResultParams moveAsync(Axis& axis, Axes&... axes);
        // void stopAsync() override;

        //template<typename... Axes>
        //void move(Axis& axis, Axes&... axes);
        //void stop() override;

        //void emergencyStop();

        void sendMove();


    protected:
        ResultParams prepareMove();
        void prepareMoveWithoutSync();
        //void tick() override; // функия тактирования двигателей, вызываемая генератором

        void (*onMoveStarted)() = nullptr; // callback перед началом движения
        void (*onMoveFinished)() = nullptr; // callback после завершения движения

        void (*onEmergensyStoped)() = nullptr; // callback после аварийной остановки

        
    
    private:
        MyCanOpen* canOpen;
        
};

template <typename... Axes>
ResultParams MoveControllerBase::moveAsync(Axis& axis, Axes&... axes)
{
    if (isMoving())
        return ResultParams();

    movingInProgress = true;
    attachAxes(axis, axes...);
    //prepareMoveWithoutSync();
    prepareMove();

    //if (onMoveStarted != nullptr)
    //        onMoveStarted();
    sendMove();
    //referenceGenerator.begin([this] { this->tick(); }, period);
    //return res;
    movingInProgress = false;
    return ResultParams();
}

/*
template <typename... Axes>
inline void MoveControllerBase::move(Axis &axis, Axes &...axes)
{
    moveAsync(axis, axes...);
    while (isMoving())
    {
        delay(1);
    }       
}
*/
}

#endif