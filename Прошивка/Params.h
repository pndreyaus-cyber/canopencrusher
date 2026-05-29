#ifndef PARAMS_H
#define PARAMS_H

#include <cstddef>
#include <vector>
#include <unordered_map>
#include <Arduino.h>
#include <optional>

#include "KinematicSolver.h"

enum struct ParamsStatus
{
    OK,
    INVALID_PARAMS,
    INCORRECT_COMMAND,
    OUT_OF_LIMITS,
};

struct ParamsStatusStruct{
    ParamsStatus status = ParamsStatus::OK;
    std::optional<String> errorMsg;
    
    String toStr()
    {
        switch (status)
        {
        case ParamsStatus::OK:
            return "OK";
        case ParamsStatus::INVALID_PARAMS:
            return "IP";
        case ParamsStatus::INCORRECT_COMMAND:
            return "IC";
        case ParamsStatus::OUT_OF_LIMITS:
            return "IL";
        default:
            return "EU";
        }
    }
};

struct SpeedAcceleration
{
    ParamsStatusStruct status;
    
    double speed = 0;
    double acceleration = 0;
};

template<std::size_t N>
struct MoveParams
{
    ParamsStatusStruct status;

    JointAngles<N> angles;
    SpeedAcceleration move;
};

struct MoveCartesianParams
{
    ParamsStatusStruct status;

    Position position;
    SpeedAcceleration move;
};

struct MotorIndices
{
    ParamsStatusStruct status;
    std::vector<uint8_t> nodeIds;
};

struct MotorIndex
{
    ParamsStatusStruct status;
    uint8_t nodeId;
};

struct PIValue
{
    ParamsStatusStruct status;
    uint8_t nodeId;
    uint16_t vp;
    uint16_t vi;
    uint16_t pp;
    uint16_t ff;
};

struct MoveToleranceWrite
{
    ParamsStatusStruct status;
    uint8_t nodeId = 0;
    uint32_t toleranceSteps = 0;
};

#endif