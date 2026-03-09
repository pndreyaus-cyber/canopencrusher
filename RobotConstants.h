#pragma once
#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H

#include <cstdint>
#include <array>
#include <functional>
#include <cstring>
#include <Arduino.h>

using callback_x6064_positionActualValue = std::function<void(uint8_t, bool, int32_t)>;
using callback_x260A_electronicGearMolecules = std::function<void(uint8_t, bool)>;
using callback_x6040_controlword = std::function<void(uint8_t, bool)>;
using callback_x6060_modesOfOperation = std::function<void(uint8_t, bool)>;
using callback_x607A_targetPosition = std::function<void(uint8_t, bool)>;
using callback_x6081_profileVelocity = std::function<void(uint8_t, bool)>;
using callback_x6083_profileAcceleration = std::function<void(uint8_t, bool)>;
using callback_TPDO1 = std::function<void(uint8_t, int32_t, uint16_t)>;
using callback_TPDO4 = std::function<void(uint8_t, int32_t, uint16_t)>;

using callback_heartbeat = std::function<void(uint8_t, uint8_t)>;

using callback_read_x6041_statusword = std::function<void(uint8_t, bool, uint16_t)>;
using callback_read_x6040_controlword = std::function<void(uint8_t, bool, uint16_t)>;

namespace RobotConstants
{
    enum InitStatus : uint8_t
    {
        ZEI_NONE = 0,
        ZEI_FAILED = 1,
        ZEI_ONGOING = 2,
        ZEI_FINISHED = 3
    };

    enum MoveStatus : uint8_t
    {
        NOT_TASKED_WITH_MOVE = 0,
        TASKED_WITH_MOVE = 1,
        MOVE_PREPARATION_FAIL = 2,
        MOVE_PREPARATION_FAIL_OUT_OF_LIMITS = 3,
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
        case InitStatus::ZEI_NONE:
            return "ZEI_NONE";
        case InitStatus::ZEI_FAILED:
            return "ZEI_FAILED";
        case InitStatus::ZEI_ONGOING:
            return "ZEI_ONGOING";
        case InitStatus::ZEI_FINISHED:
            return "ZEI_FINISHED";
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
        //const String MOVE_ABSOLUTE = "MAJ"; // Not implemented
        const String MOVE_ABSOLUTE_PERCENT = "MAJ";
        const String MOVE_RELATIVE = "MRJ"; // Not implemented
        const String ECHO = "ECH";
        const String MOTOR_STATUS = "RMS";
        const String ZERO_INITIALIZE = "ZEI";
        const String REQUEST_POSITION = "RPP";
        const String PREPAREMOVE_TEST = "PMT";
        const String REQUEST_POSITION_ANGLES = "RPA";
        constexpr int COMMAND_LEN = 3;
        const float MIN_SPEED_UNITS = 0.0f;
        const float MAX_SPEED_UNITS = 100.0f;
        const float MIN_ACCELERATION_UNITS = 0.0f;
        const float MAX_ACCELERATION_UNITS = 100.0f;
    }

    // Robot specifications
    namespace Robot
    {
        constexpr uint8_t AXES_COUNT = 5;
        constexpr uint8_t MAX_AXES_COUNT = 6;
        constexpr uint8_t MIN_NODE_ID = 'A';
        constexpr uint8_t MAX_NODE_ID = (AXES_COUNT == 0) ? MIN_NODE_ID : static_cast<uint8_t>(MIN_NODE_ID + AXES_COUNT - 1);
        constexpr uint8_t AXIS_IDENTIFIER_CHAR = 'J';
        constexpr uint32_t CONTROL_LOOP_HZ = 1000;
        constexpr uint32_t CAN_BAUD_RATE = 1000000; // 1 Mbps
        constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
        constexpr uint32_t HEARTBEAT_TIMEOUT_MS = static_cast<uint32_t>(HEARTBEAT_INTERVAL_MS * 2);
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

        // Motor parameters
        constexpr uint16_t VELOCITY_CONTROL_PARAM = 0x60F9;
        constexpr uint16_t POSITION_CONTROL_PARAM = 0x60FB;

        // Driver specific
        constexpr uint16_t MODBUS_ENABLE = 0x2600;
        constexpr uint16_t DRIVER_ENABLE = 0x2601;
        constexpr uint16_t ELECTRONIC_GEAR_MOLECULES = 0x260A;
        constexpr uint16_t ELECTRONIC_GEAR_DENOMINATOR = 0x260B;
        constexpr uint16_t DEVICE_ADDRESS = 0x2615;

        // Default subindex
        constexpr uint8_t DEFAULT_SUBINDEX = 0x00;
    }

    // Axis configuration
    namespace Axis
    {
        constexpr int32_t STEPS_PER_MOTOR_REV = 32768;
        constexpr double UNITS_PER_OUTPUT_SHAFT_REV = 360; // 1 revolution of output shaft corresponds to 360 degrees
        constexpr int GEAR_RATIO = 50;
        constexpr double UNITS_PER_MOTOR_REV = UNITS_PER_OUTPUT_SHAFT_REV / GEAR_RATIO; // 1 revolution of motor corresponds to UNITS_PER_MOTOR_REV degrees
        constexpr double DEFAULT_MAX_LIMITS[] = {170.0, 90.0, 135.0, 180.0, 110.0, 0.0}; // Max velocity in degrees per second for each axis
        constexpr double DEFAULT_MIN_LIMITS[] = {-170.0, -45.0, -135.0, -180.0, -110.0, 0.0}; // Min velocity in degrees per second for each axis
        constexpr double LIMIT_TOLERANCE = 0.1; // Tolerance in degrees for limit checking
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

        constexpr double MINIMUM_ABSOLUTE_ANGLE = 9.0/40960.0; // 0.0002197265
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
    namespace Status
    {
        const String OK = "OK";
        const String COMMAND_FULL_FAIL = "FF";
        const String COMMAND_PARTIAL_FAIL = "PF";
        const String INCORRECT_COMMAND = "IC";
        const String INVALID_PARAMS = "IP";
        const String UNKNOWN_ERROR = "UE";
        const String LOGIC_ERROR = "LE";
        const String NOT_IMPLEMENTED = "NI";
        const String NOT_INITIALIZED = "NZ";
        const String OTHER_COMMAND_IN_PROGRESS = "OP";
    }

} // namespace RobotConstants

#endif