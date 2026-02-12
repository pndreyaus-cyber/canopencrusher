#pragma once
#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H

#include <cstdint>
#include <array>
#include <functional>
#include <Arduino.h>

using callback_x6064_positionActualValue = std::function<void(uint8_t, bool, int32_t)>;
using callback_x260A_electronicGearMolecules = std::function<void(uint8_t, bool)>;
using callback_x6040_controlword = std::function<void(uint8_t, bool)>;
using callback_x6060_modesOfOperation = std::function<void(uint8_t, bool)>;
using callback_x607A_targetPosition = std::function<void(uint8_t, bool)>;
using callback_x6041_statusword = std::function<void(uint8_t, bool, uint16_t)>;

using callback_heartbeat = std::function<void(uint8_t, uint8_t)>;

namespace RobotConstants
{
    enum InitStatus : uint8_t
    {
        ZEI_NONE = 0,
        ZEI_FAILED = 1,
        ZEI_ONGOING = 2,
        ZEI_FINISHED = 3
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
        const String MOVE_ABSOLUTE = "MAJ";
        const String MOVE_RELATIVE = "MRJ";
        const String ECHO = "ECH";
        const String MOTOR_STATUS = "RMS";
        const String ZERO_INITIALIZE = "ZEI";
        const String REQUEST_POSITION = "RPP";
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
        constexpr uint8_t MAX_NODE_ID = MIN_NODE_ID + AXES_COUNT;
        constexpr uint8_t AXIS_IDENTIFIER_CHAR = 'J';
        constexpr uint32_t CONTROL_LOOP_HZ = 1000;
        constexpr uint32_t CAN_BAUD_RATE = 1000000; // 1 Mbps
        constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
        constexpr uint32_t HEARTBEAT_TIMEOUT_MS = static_cast<uint32_t>(HEARTBEAT_INTERVAL_MS * 5);
    }

    // CANopen communication constants
    namespace CANOpen
    {
        constexpr uint32_t COB_ID_SYNC = 0x080;
        constexpr uint32_t COB_ID_NMT = 0x000;
        constexpr uint32_t COB_ID_HEARTBEAT_BASE = 0x700;
        constexpr uint32_t COB_ID_SDO_SERVER_BASE = 0x600;
        constexpr uint32_t COB_ID_SDO_CLIENT_BASE = 0x580;
        constexpr uint32_t COB_ID_PDO_BASE = 0x180;

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

    // Control parameters
    namespace Control
    {
        constexpr float DEFAULT_SPEED = 1000.0f;
        constexpr float DEFAULT_ACCELERATION = 500.0f;
        constexpr float DEFAULT_DECELERATION = 500.0f;
        constexpr uint32_t DEFAULT_PROFILE_VELOCITY = 1000;
        constexpr uint32_t DEFAULT_PROFILE_ACCELERATION = 500;
        constexpr uint16_t DEFAULT_CONTROLWORD = 0x000F;
        constexpr uint8_t DEFAULT_MODE_POSITION = 1;
        constexpr uint8_t DEFAULT_MODE_VELOCITY = 3;
    }

    // Axis configuration
    namespace Axis
    {
        constexpr uint32_t DEFAULT_STEPS_PER_REVOLUTION = 32768;
        constexpr double DEFAULT_UNITS_PER_REVOLUTION = 7.2; //
        constexpr double DEFAULT_MIN_LIMIT = -1000.0;
        constexpr double DEFAULT_MAX_LIMIT = 1000.0;
        constexpr bool DEFAULT_USE_LIMITS = false;
    }

    // Buffer sizes
    namespace Buffers
    {
        constexpr size_t CAN_FRAME_SIZE = 8;
        constexpr size_t MAX_CAN_MESSAGE_LEN = 8;
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
        const String INVALID_NODE_ID = "IN";
    }

} // namespace RobotConstants

#endif // ROBOT_CONSTANTS_H
