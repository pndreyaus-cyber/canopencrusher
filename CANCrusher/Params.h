#ifndef PARAMS_H

#define PARAMS_H

#include <cstddef>
#include <vector>
#include <unordered_map>
#include <Arduino.h>
#include <optional>

enum struct ParamsStatus
{
    OK,
    INVALID_PARAMS,
};

struct ParamsStatusStruct{
    ParamsStatus status = ParamsStatus::OK;
    std::optional<String> errorMsg;
};

template<std::size_t N>
struct MoveParams
{
    ParamsStatusStruct status;

    double movementUnits[N] = {};
    double speed = 0; // It should be in percent [0, 1]
    double acceleration = 0; // It should be in percent [0, 1]
};

struct MotorIndices
{
    ParamsStatusStruct status;
    std::vector<uint8_t> nodeIds;
};

#endif