#ifndef MOVECONTROLLERBASE_H
#define MOVECONTROLLERBASE_H

#include "ControllerBase.h"
#include "CanOpen.h"
#include "Params.h"
#include <string>

namespace StepDirController{ 
class MoveControllerBase : public ControllerBase{
    public:
        //MoveControllerBase(CanOpen *canOpen);
        bool start(CanOpen* canOpen){
            this->canOpen = canOpen;
            return true;
        }
        
        void setRegularSpeedUnits(double speed) override; // настройка крейсерской скорости в единицах измерения в секунду (градусы в секунду)
        void setAccelerationUnits(double acceleration) override; // настройка ускорения в единицах измерения в секунду^2 (градусы в секунду^2)

        double getRegularSpeedUnits() const;
        double getAccelerationUnits() const;

        void setOnMoveStarted(void (*callback)());
        void setOnMoveFinished(void (*callback)());
        void setOnEmergensyStoped(void (*callback)());


        template<typename... Axes>
        void moveAsync(Axis& axis, Axes&... axes);

        // TODO: добавить метод, например, void tick(), вызывать его из loop в ino-файле
        // метод tick() должен вызывать метод чтения (который нужно написать) у CanOpen (который сам взаимодействует с шиной CAN)
        // CanOpen может возвращать значения через колбэки
        // полученные значения через колбэки сохранять в Axis

        //TODO: метод tick должен рассылать запросы по двигателям о их состоянии (напряжение, температура, т.д. - все что есть по протоколу)


    protected:
        void prepareMove();
        void prepareMoveWithoutSync();

        void (*onMoveStarted)() = nullptr; // callback перед началом движения
        void (*onMoveFinished)() = nullptr; // callback после завершения движения

        void (*onEmergensyStoped)() = nullptr; // callback после аварийной остановки
        
    private:
        CanOpen* canOpen;
        void sendMove();
};

template <typename... Axes>
void MoveControllerBase::moveAsync(Axis& axis, Axes&... axes)
{
    if (isMoving())
        return;

    movingInProgress = true;
    attachAxes(axis, axes...);
    prepareMove();
    sendMove();
    movingInProgress = false; // TODO: снимать флаг при получении ответа от двигателей о завершении движения
}

}

#endif