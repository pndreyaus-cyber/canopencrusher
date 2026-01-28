#ifndef PARAMS_H

#define PARAMS_H

#include <cstddef>

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
	double movementUnits[N];
	double speed;
	double acceleration;
	//double deceleration;
};

struct PositionParams
{
	ParamsStatus status;
	int32_t currentPosition;
	uint8_t nodeId; 
};

#endif