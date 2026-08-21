#pragma once
#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H

#include <cstdint>
#include <array>
#include <functional>
#include <cstring>
#include <Arduino.h>

#define ROBOT_CAN_RX PA11 // Used to be PB8
#define ROBOT_CAN_TX PA12 // Used to be PB9

#define GRIPPER_PIN PC13

#define PIN_OUT_NUM 1
// const uint32_t userPinMap[] = {
//   PB13,
//   PB14,
//   PB15,
//   PB3,
//   PB4,
//   PB12,
//   PB6,
//   PB7,
//   PC13,
// };

const uint32_t userPinMap[] = {
    PB0,
    //PB13,
   // PB14,
    //PB15,
    //PB3,
    //PB4,
    //PC13,
    //PB7
};

using callback_x6064_positionActualValue = std::function<void(uint8_t, bool, int32_t)>;
using callback_x260A_electronicGearMolecules = std::function<void(uint8_t, bool)>;
using callback_x6040_controlword = std::function<void(uint8_t, bool)>;
using callback_x6060_modesOfOperation = std::function<void(uint8_t, bool)>;
using callback_x607A_targetPosition = std::function<void(uint8_t, bool)>;
using callback_x6081_profileVelocity = std::function<void(uint8_t, bool)>;
using callback_x6083_profileAcceleration = std::function<void(uint8_t, bool)>;
using callback_x2614_dataSaveFlag = std::function<void(uint8_t, bool)>;
using callback_TPDO1 = std::function<void(uint8_t, int32_t, uint16_t)>;
using callback_TPDO4 = std::function<void(uint8_t, int32_t, uint16_t)>;

using callback_heartbeat = std::function<void(uint8_t, uint8_t)>;

using callback_read_x6041_statusword = std::function<void(uint8_t, bool, uint16_t)>;
using callback_read_x6040_controlword = std::function<void(uint8_t, bool, uint16_t)>;

using callback_read_PI_controller = std::function<void(uint8_t, uint16_t, uint8_t, bool, int16_t)>; // node-id, index, subindex, success, value

using callback_read_x2614_dataSaveFlag = std::function<void(uint8_t, bool, uint8_t)>;

using callback_x60F9_01_VP = std::function<void(uint8_t, bool)>;
using callback_x60F9_02_VI = std::function<void(uint8_t, bool)>;
using callback_x60FB_01_PP = std::function<void(uint8_t, bool)>;
using callback_x60FB_02_FF = std::function<void(uint8_t, bool)>;

namespace RobotConstants
{
    enum InitStatus : uint8_t
    {
        ZOE_NONE = 0,
        ZOE_FAILED = 1,
        ZOE_ONGOING = 2,
        ZOE_FINISHED = 3
    };

    enum MoveStatus : uint8_t
    {
        NOT_TASKED_WITH_MOVE = 0,
        TASKED_WITH_MOVE = 1,
        MOVE_PREPARATION_FAIL = 2,
        MOVE_PREPARATION_SUCCESS = 4,
        READY_TO_MOVE = 5,
        MOVING = 6,
        MOVE_FAIL = 7,
        MOVE_FINISHED = 8,
        MOVE_SUCCESS = 9,
    };

    enum AxisStatus : uint8_t
    {
        NOT_ALIVE = 0,
        ALIVE_BUT_NOT_INITIALIZED = 1,
        ALIVE = 2
    };

    enum MoveUnits : uint8_t
    {
        UNITS_PERCENT = 0,
        UNITS_MM_PER_SEC = 1,
        UNITS_DEG_PER_SEC = 2,
    };

    inline const char *initStatusToString(InitStatus status)
    {
        switch (status)
        {
        case InitStatus::ZOE_NONE:
            return "ZOE_NONE";
        case InitStatus::ZOE_FAILED:
            return "ZOE_FAILED";
        case InitStatus::ZOE_ONGOING:
            return "ZOE_ONGOING";
        case InitStatus::ZOE_FINISHED:
            return "ZOE_FINISHED";
        default:
            return "UNKNOWN";
        }
    }

    // Physical and mathematical constants
    namespace Math
    {
        constexpr double SECONDS_IN_MINUTE = 60.0;
    }

    // Command identifiers sent to the robot controller
    namespace Commands
    {
        const String MOVE_ABSOLUTE_JOINT = "MAJ";
        const String ECHO = "ECH";
        const String REQUEST_MOTOR_STATUS = "RMS";
        const String ZERO_OUT_ENCODER = "ZOE";
        const String JOINT_POSITIONS_STEPS = "JPS";
        const String JOINT_POSITIONS_DEGREES = "JPD";
        const String ROBOT_POSITION_CARTESIAN = "RPC";
        const String PI_CONTROLLER_READ = "PIR";
        const String PI_CONTROLLER_WRITE = "PIW";
        const String GRIPPER_WRITE = "GRW";
        const String GRIPPER_READ = "GRR";
        const String MOVE_HOME_JOINT = "MHJ";
        const String HOME_JOINT_WRITE = "WHJ";
        const String HOME_JOINT_READ = "RHJ";
        const String MOVE_TIMEOUT_WRITE = "WMT";
        const String MOVE_TIMEOUT_READ = "RMT";
        const String MOVE_ABSOLUTE_CARTESIAN = "MAC";
        const String PIN_OUT_WRITE = "POW";
        const String PIN_OUT_READ = "POR";
        const String READ_MODEL = "RMD";
        const String READ_SOFTWARE_VERSION = "RSV";
        const String READ_SERIAL_NUMBER = "RSN";
        const String WRITE_SERIAL_NUMBER = "WSN";
        const String MOVE_TOLERANCE_WRITE = "MTW";
        const String MOVE_TOLERANCE_READ = "MTR";

        constexpr int COMMAND_LEN = 3;
    }

    // Robot specifications
    namespace Robot
    {
        constexpr uint8_t AXES_COUNT = 1;
        constexpr uint8_t MAX_AXES_COUNT = 6;
        constexpr uint8_t MIN_NODE_ID = 'A';
        constexpr uint8_t MAX_NODE_ID = (AXES_COUNT == 0) ? MIN_NODE_ID : static_cast<uint8_t>(MIN_NODE_ID + AXES_COUNT - 1);
        constexpr uint8_t AXIS_IDENTIFIER_CHAR = 'J';
        constexpr uint32_t CAN_BAUD_RATE = 1000000; // 1 Mbps
        constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
        constexpr uint32_t HEARTBEAT_TIMEOUT_MS = static_cast<uint32_t>(HEARTBEAT_INTERVAL_MS * 2);

        const String MODEL = "MAI-1";
        const String SOFTWARE_VERSION = "SW-1";
        const int SERIAL_NUMBER_MAX_LENGTH = 24;

    }

    // CANopen communication constants
    namespace CANOpen
    {
        constexpr uint32_t COB_ID_SYNC = 0x080;
        constexpr uint32_t COB_ID_NMT = 0x000;
        constexpr uint32_t COB_ID_HEARTBEAT_BASE = 0x700;
        constexpr uint32_t COB_ID_SDO_SERVER_BASE = 0x600;
        constexpr uint32_t COB_ID_SDO_CLIENT_BASE = 0x580;
        constexpr uint32_t COB_ID_TPDO1_BASE = 0x180;
        constexpr uint32_t COB_ID_RPDO1_BASE = 0x200;
        constexpr uint32_t COB_ID_TPDO4_BASE = 0x480;
        constexpr uint32_t COB_ID_RPDO4_BASE = 0x500;

        // PDO mapping
        constexpr uint8_t PDO_COUNT = 4;
        constexpr uint8_t PDO_MAPPING_MAX_ENTRIES = 8;

        // SDO
        constexpr uint8_t MAX_SDO_WRITE_DATA_SIZE = 4; // Max 4 bytes for expedited SDO write
        constexpr uint8_t REGISTER_INDEX_SIZE = 2;     // 2 bytes for index
        constexpr uint8_t REGISTER_SUBINDEX_SIZE = 1;  // 1 byte for subindex
        constexpr uint8_t SDO_FUNCTION_CODE_SIZE = 1;  // 1 byte for function code
        constexpr uint8_t HEADER_SIZE = SDO_FUNCTION_CODE_SIZE + REGISTER_INDEX_SIZE + REGISTER_SUBINDEX_SIZE;
    }

    // Object dictionary indices (from OD.h)
    namespace ODIndices
    {
        constexpr uint16_t DEVICE_TYPE = 0x1000;
        constexpr uint16_t ERROR_REGISTER = 0x1001;
        constexpr uint16_t SYNC_COB_ID = 0x1005;
        constexpr uint16_t COMM_CYCLE_PERIOD = 0x1006;
        constexpr uint16_t CONSUMER_HEARTBEAT_TIME = 0x1016;
        constexpr uint16_t PRODUCER_HEARTBEAT_TIME = 0x1017;
        constexpr uint16_t IDENTITY = 0x1018;
        constexpr uint16_t SERVER_SDO_PARAM = 0x1200;
        constexpr uint16_t CLIENT_SDO_PARAM = 0x1280;

        // PDO parameters
        constexpr uint16_t RPDO_PARAM_BASE = 0x1400;
        constexpr uint16_t TPDO_PARAM_BASE = 0x1800;
        constexpr uint16_t RPDO_MAPPING_BASE = 0x1600;
        constexpr uint16_t TPDO_MAPPING_BASE = 0x1A00;

        // Motor control
        constexpr uint16_t CONTROLWORD = 0x6040;
        constexpr uint16_t STATUSWORD = 0x6041;
        constexpr uint16_t MODES_OF_OPERATION = 0x6060;
        constexpr uint16_t POSITION_ACTUAL_VALUE = 0x6064;
        constexpr uint16_t VELOCITY_ACTUAL_VALUE = 0x606C;
        constexpr uint16_t CURRENT_ACTUAL_VALUE = 0x6078;
        constexpr uint16_t TARGET_POSITION = 0x607A;
        constexpr uint16_t PROFILE_VELOCITY = 0x6081;
        constexpr uint16_t PROFILE_ACCELERATION = 0x6083;
        constexpr uint16_t TARGET_VELOCITY = 0x60FF;

        // Control parameters
        constexpr uint16_t VELOCITY_LOOP_CONTROL = 0x60F9;
        constexpr uint8_t VELOCITY_KP_SUBINDEX = 0x01;
        constexpr uint8_t VELOCITY_KI_SUBINDEX = 0x02;

        constexpr uint16_t POSITION_LOOP_CONTROL = 0x60FB;
        constexpr uint8_t POSITION_KP_SUBINDEX = 0x01;
        constexpr uint8_t FEEDFORWARD = 0x02;

        // Parameter saving
        constexpr uint16_t DATA_SAVE_FLAG = 0x2614;

        // Driver specific
        constexpr uint16_t MODBUS_ENABLE = 0x2600;
        constexpr uint16_t DRIVER_ENABLE = 0x2601;
        constexpr uint16_t ELECTRONIC_GEAR_MOLECULES = 0x260A;
        constexpr uint16_t ELECTRONIC_GEAR_DENOMINATOR = 0x260B;
        constexpr uint16_t DEVICE_ADDRESS = 0x2615;

        // Default subindex
        constexpr uint8_t DEFAULT_SUBINDEX = 0x00;
    }

    namespace MotorControlLimits
    {
        constexpr int16_t MIN_VELOCITY_P = 0;     // Min velocity in device-specific units for P gain calculation
        constexpr int16_t MAX_VELOCITY_P = 10000; // Max velocity in device-specific units for P gain calculation

        constexpr int16_t MIN_VELOCITY_I = 2;    // Min velocity in device-specific units for I gain calculation
        constexpr int16_t MAX_VELOCITY_I = 2000; // Max velocity in device-specific units for I gain calculation

        constexpr int16_t MIN_POSITION_P = 60;    // Min velocity in device-specific units for P gain calculation
        constexpr int16_t MAX_POSITION_P = 30000; // Max velocity in device-specific units for P gain calculation

        constexpr int16_t MIN_FEEDFORWARD = 0;    // Min velocity in device-specific units for feedforward factor calculation
        constexpr int16_t MAX_FEEDFORWARD = 3924; // Max velocity in device-specific units for feedforward factor calculation
    }

    // Axis configuration
    namespace Axis
    {
        constexpr int32_t STEPS_PER_MOTOR_REV = 32768;
        constexpr double UNITS_PER_OUTPUT_SHAFT_REV = 360; // 1 revolution of output shaft corresponds to 360 degrees
        constexpr int GEAR_RATIO = 50;
        constexpr double UNITS_PER_MOTOR_REV = UNITS_PER_OUTPUT_SHAFT_REV / GEAR_RATIO;     // 1 revolution of motor corresponds to UNITS_PER_MOTOR_REV degrees
        constexpr double DEFAULT_MAX_LIMITS[] = {360, 360.0, 360.0, 360.0, 360.0, 360.0};      // Max velocity in degrees per second for each axis
        constexpr double DEFAULT_MIN_LIMITS[] = {-360, -360.0, -360.0, -360.0, -360.0, -360.0}; // Min velocity in degrees per second for each axis
        constexpr double LIMIT_TOLERANCE = 0.1;                                             // Tolerance in degrees for limit checking
        constexpr uint32_t DEFAULT_MAJ_MOVE_TOLERANCE_STEPS = 1000;                           // Host MAJ completion: |6064 - 607A| <= this (drive units)
    }

    // Control parameters
    namespace Control
    {
        constexpr uint32_t MAXIMUM_PROFILE_VELOCITY_IN_RPM = 3000;
        constexpr double MAXIMUM_PROFILE_VELOCITY_IN_DEG_PER_S = 360;
        constexpr uint32_t MAXIMUM_PROFILE_VELOCITY_IN_STEPS_PER_SECOND = 1638400; // Corresponds to 3000 RPM for a motor with 32768 steps per revolution
        constexpr double MAXIMUM_PROFILE_VELOCITY_IN_PERCENT = 1.0;

        constexpr uint32_t MAXIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S = 65535;
        constexpr double MAXIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2 = 7864.2;
        constexpr uint32_t MAXIMUM_PROFILE_ACCELERATION_IN_STEPS_PER_SECOND2 = 35790848; // Corresponds to 65535 RPM/s for a motor with 32768 steps per revolution
        constexpr double MAXIMUM_PROFILE_ACCELERATION_IN_PERCENT = 1.0;

        constexpr uint32_t MINIMUM_PROFILE_VELOCITY_IN_RPM = 1;
        constexpr double MINIMUM_PROFILE_VELOCITY_IN_DEG_PER_S = 0.12; // Corresponds to 1 RPM for a motor with 32768 steps per revolution
        constexpr double MINIMUM_PROFILE_VELOCITY_IN_PERCENT = 0.000333333;
        constexpr uint32_t MINIMUM_PROFILE_VELOCITY_IN_STEPS_PER_SEC = Axis::STEPS_PER_MOTOR_REV * MINIMUM_PROFILE_VELOCITY_IN_RPM / Math::SECONDS_IN_MINUTE;

        constexpr uint32_t MINIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S = 1;
        constexpr double MINIMUM_PROFILE_ACCELERATION_IN_DEG_PER_S2 = 0.12; // Corresponds to 1 RPM/s for a motor with 32768 steps per revolution
        constexpr double MINIMUM_PROFILE_ACCELERATION_IN_PERCENT = 0.000015259;
        constexpr uint32_t MINIMUM_PROFILE_ACCELERATION_IN_STEPS_PER_SEC2 = Axis::STEPS_PER_MOTOR_REV * MINIMUM_PROFILE_ACCELERATION_IN_RPM_PER_S / Math::SECONDS_IN_MINUTE;

        constexpr double MINIMUM_ABSOLUTE_ANGLE = 9.0 / 40960.0; // 0.0002197265

        /** Wall-clock limit for a single MAJ (host-side); default when EEPROM has no saved value. */
        constexpr uint32_t MAJ_MOVE_TIMEOUT_MS = 10000;

        constexpr uint32_t MIN_MAJ_MOVE_TIMEOUT_SEC = 1;
        constexpr uint32_t MAX_MAJ_MOVE_TIMEOUT_SEC = 3600;
    }

    // Buffer sizes
    namespace Buffers
    {
        constexpr size_t CAN_FRAME_SIZE = 8;
        constexpr size_t MAX_CAN_MESSAGE_LEN = 8;
        constexpr size_t SERIAL_OUT_QUEUE_CAPACITY = 128;
        constexpr size_t SERIAL_MESSAGE_CAPACITY = 128;
    }
    // Status codes
    namespace Result // Status --> Result
    {
        const String OK = "OK";
        const String FAIL = "FF";
        const String INCORRECT_COMMAND = "IC";
        const String INVALID_PARAMS = "IP";
        const String ERROR_UNKNOWN = "EU";
        const String NOT_INITIALIZED = "NZ";
        const String OPERATION_FORBIDDEN = "OF";
        const String UNSUPPORTED_COMMAND = "UC";
        const String NO_DATA = "ND";
        const String TIMEOUT = "TO";
    }

    namespace Eeprom
    {
        struct SerialNumber
        {
            uint32_t magic;
            char serial[25];
        };

        struct Home
        {
            uint32_t magic;
            float joints[6];
        };

        struct MoveTimeout
        {
            uint32_t magic;
            uint32_t timeoutSec;
        };

        const uint32_t SERIAL_MAGIC = 0x534E3031;     // "SN01"
        const uint32_t HOME_JOINT_MAGIC = 0x484A3031; // "HJ01"
        const uint32_t MOVE_TIMEOUT_MAGIC = 0x4D543031; // "MT01"

        const int SERIAL_NUMBER_ADDR = 0;
        const int HOME_JOINT_ADDR = SERIAL_NUMBER_ADDR + sizeof(SerialNumber);
        const int MOVE_TIMEOUT_ADDR = HOME_JOINT_ADDR + sizeof(Home);

    }

} // namespace RobotConstants

#endif