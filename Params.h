#ifndef PARAMS_H

#define PARAMS_H

#include <cstddef>
#include <vector>
#include <unordered_map>
#include <Arduino.h>

enum struct ParamsStatus
{
    OK,
    INCORRECT_COMMAND,
    INVALID_PARAMS,
};

template<std::size_t N>
struct MoveParams
{
    ParamsStatus status = ParamsStatus::OK;
    String errorMsg;
    double movementUnits[N] = {};
    uint32_t speed = 0;
    uint32_t acceleration = 0;
};

struct MotorIndices
{
    ParamsStatus status;
    std::vector<uint8_t> nodeIds;
    String errorMsg;
    String errorCode;
};

#endif