#pragma once

#include "Arduino.h"

#define DBG_LEVEL_ERROR 1u
#define DBG_LEVEL_WARN 2u
#define DBG_LEVEL_INFO 3u
#define DBG_LEVEL_VERBOSE 4u

#define DBG_GROUP_AXIS (1u << 0)
#define DBG_GROUP_MOVE (1u << 1)
#define DBG_GROUP_CANOPEN (1u << 2)
#define DBG_GROUP_COMMAND (1u << 3)
#define DBG_GROUP_HEARTBEAT (1u << 4)
#define DBG_GROUP_ZEI (1u << 5)
#define DBG_GROUP_SERIAL (1u << 6)
#define DBG_GROUP_ALL 0xFFFFu

#define DBG_LEVEL_SHIFT 16u
#define DBG_GROUP_MASK 0xFFFFu

#define DBG_CONFIG(level, groups) (((uint32_t)(level) << DBG_LEVEL_SHIFT) | ((uint32_t)(groups)))

#include "DebugConfig.h"

#ifndef DEBUG_CONFIG
#define DEBUG_CONFIG 0u
#endif

inline uint32_t dbgConfigLevel(uint32_t config)
{
    return (config >> DBG_LEVEL_SHIFT);
}

inline uint32_t dbgConfigGroups(uint32_t config)
{
    return (config & DBG_GROUP_MASK);
}

inline const char *dbgLevelTag(uint32_t level)
{
    switch (level)
    {
    case DBG_LEVEL_ERROR:
        return "ERROR";
    case DBG_LEVEL_WARN:
        return "WARN";
    case DBG_LEVEL_INFO:
        return "INFO";
    case DBG_LEVEL_VERBOSE:
        return "VERBOSE";
    default:
        return "DBG";
    }
}

extern void addDataToOutQueue(String data);

#define DBG_ENABLED(level, group) ((dbgConfigLevel(DEBUG_CONFIG) >= (level)) && ((dbgConfigGroups(DEBUG_CONFIG) & (group)) != 0u))

#define DBG_LOG(level, group, msg)     \
    do                                 \
    {                                  \
        if (DBG_ENABLED(level, group)) \
        {                              \
            addDataToOutQueue(String("[") + dbgLevelTag(level) + "] " + String(msg)); \
        }                              \
    } while (0)

#define DBG_ERROR(group, msg) DBG_LOG(DBG_LEVEL_ERROR, group, msg)
#define DBG_WARN(group, msg) DBG_LOG(DBG_LEVEL_WARN, group, msg)
#define DBG_INFO(group, msg) DBG_LOG(DBG_LEVEL_INFO, group, msg)
#define DBG_VERBOSE(group, msg) DBG_LOG(DBG_LEVEL_VERBOSE, group, msg)
