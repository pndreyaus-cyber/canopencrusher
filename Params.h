#ifndef PARAMS_H

#define PARAMS_H

#include <cstddef>
#include <vector>
#include <Arduino.h>

enum struct ParamsStatus
{
    OK,
    INCORRECT_COMMAND,
    INVALID_PARAMS,
};

enum struct CanResult
{
    SUCCESS = 0,
    INVALID_LENGTH,
    NULL_DATA_POINTER,
    SEND_FAILED,
    NOT_RECEIVED,
    RECEIVED
};

template <std::size_t N>
struct MoveParams
{
    ParamsStatus status;
    String errorMsg;
    double movementUnits[N];
    double speed;
    double acceleration;
    // double deceleration;
};

struct MotorIndices
{
    ParamsStatus status;
    std::vector<uint8_t> nodeIds;
    String errorMsg;
    String errorCode;
};

#endif