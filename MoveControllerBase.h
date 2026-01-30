#ifndef MOVECONTROLLERBASE_H

#define MOVECONTROLLERBASE_H

#include <string>
#include <unordered_map>
#include "CanOpen.h"
#include "Params.h"
#include "Axis.h"


namespace StepDirController{ 
class MoveControllerBase {
    public:
        bool start(CanOpen* canOpen, uint8_t axesCnt);

        uint8_t getAxesCount() const { return axesCnt; }
        Axis& getAxis(uint8_t nodeId) { return axes.at(nodeId);}
        
        void setRegularSpeedUnits(double speed); // настройка крейсерской скорости в единицах измерения в секунду (градусы в секунду)
        void setAccelerationUnits(double acceleration); // настройка ускорения в единицах измерения в секунду^2 (градусы в секунду^2)

        double getRegularSpeedUnits() const;
        double getAccelerationUnits() const;

        void move();
        

        // TODO: добавить метод, например, void tick(), вызывать его из loop в ino-файле
        // метод tick() должен вызывать метод чтения (который нужно написать) у CanOpen (который сам взаимодействует с шиной CAN)
        // CanOpen может возвращать значения через колбэки
        // полученные значения через колбэки сохранять в Axis

        //TODO: метод tick должен рассылать запросы по двигателям о их состоянии (напряжение, температура, т.д. - все что есть по протоколу)


    protected:
        void prepareMove();
        
    private:
        CanOpen* canOpen;
        std::unordered_map<uint8_t, Axis> axes;
        bool initialized = false;
        uint8_t axesCnt = 0;

        double regularSpeedUnits = 1.0f; // The speed of the maximum moving axis in percent of full speed (1.0 = 100%)
        double accelerationUnits = 1.0f; // The acceleration of the maximum moving axis in percent of full acceleration (1.0 = 100%)

        void sendMove();

        void positionUpdateCallback(uint8_t nodeId, int32_t position);
};

}

#endif
