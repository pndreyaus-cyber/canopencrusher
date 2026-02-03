#ifndef MOVECONTROLLERBASE_H

#define MOVECONTROLLERBASE_H

#include <string>
#include <unordered_map>
#include <type_traits>
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

        void startZeroInitializationAllAxes();
        void startZeroInitializationSingleAxis(uint8_t nodeId);
        void setWorkMode(uint8_t nodeId, uint8_t mode);
        void setControlWord(uint8_t nodeId, uint16_t controlWord);


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

        bool zeroInitializeSingleAxis = true;
        uint8_t axisToInitialize = 0;

        double regularSpeedUnits = 1.0f; // The speed of the maximum moving axis in percent of full speed (1.0 = 100%)
        double accelerationUnits = 1.0f; // The acceleration of the maximum moving axis in percent of full acceleration (1.0 = 100%)

        void sendMove();

        void positionUpdate(uint8_t nodeId, int32_t position);

        template <
            typename ResetCurrentCallback,
            typename OnFailureResetCallback,
            typename OnSuccessBeforeNext,
            typename SetNextCallback,
            typename SendNext,
            typename OnSendFailResetCallback>
        bool zeroInitialize_advanceStep(
            uint8_t nodeId,
            bool success,
            const char* stepFailMsg,
            ResetCurrentCallback resetCurrentCallback,
            OnFailureResetCallback onFailureResetCallback,
            OnSuccessBeforeNext onSuccessBeforeNext,
            SetNextCallback setNextCallback,
            SendNext sendNext,
            const char* sendFailMsg,
            OnSendFailResetCallback onSendFailResetCallback) {
            if constexpr (!std::is_same_v<ResetCurrentCallback, std::nullptr_t>) {
                resetCurrentCallback(nodeId);
            }

            if (!success) {
                if (stepFailMsg != nullptr) {
                    Serial2.println(String(stepFailMsg) + String(nodeId));
                }
                axes[nodeId].initStatus = ZEI_FAILED;
                if constexpr (!std::is_same_v<OnFailureResetCallback, std::nullptr_t>) {
                    onFailureResetCallback(nodeId);
                }
                zeroInitialize_finalResult();
                return false;
            }

            if constexpr (!std::is_same_v<OnSuccessBeforeNext, std::nullptr_t>) {
                onSuccessBeforeNext(nodeId);
            }

            if constexpr (!std::is_same_v<SetNextCallback, std::nullptr_t>) {
                setNextCallback(nodeId);
            }

            if (!sendNext(nodeId)) {
                if (sendFailMsg != nullptr) {
                    Serial2.println(String(sendFailMsg) + String(nodeId));
                }
                axes[nodeId].initStatus = ZEI_FAILED;
                if constexpr (!std::is_same_v<OnSendFailResetCallback, std::nullptr_t>) {
                    onSendFailResetCallback(nodeId);
                }
                zeroInitialize_finalResult();
                return false;
            }

            return true;
        }

// Callbacks
        void zeroInitialize_firstWriteTo_0x260A(uint8_t nodeId, bool success);
        void zeroInitialize_secondWriteTo_0x260A(uint8_t nodeId, bool success);
        void zeroInitialize_firstWriteTo_0x6040(uint8_t nodeId, bool success);
        void zeroInitialize_writeTo_0x6060(uint8_t nodeId, bool success);
        void zeroInitialize_requestPosition_0x6064(uint8_t nodeId, bool success, int32_t position);
        void zeroInitialize_secondWriteTo_0x6040(uint8_t nodeId, bool success);
        void zeroInitialize_writeTo_0x607A(uint8_t nodeId, bool success);
        void zeroInitialize_requestStatusword_0x6041(uint8_t nodeId, bool success, uint16_t statusWord);

        void zeroInitialize_finalResult();

        void regularHeartbeatCallback(uint8_t nodeId, uint8_t status);
        void regularPositionActualValueCallback(uint8_t nodeId, bool success, int32_t position);
};

}

#endif
