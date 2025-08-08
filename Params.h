#ifndef PARAMS_H
#define PARAMS_H
enum struct ParamsStatus
{
	OK,
	INCORRECT_COMMAND,
	INVALID_PARAMS,
};

struct MoveParams
{
	ParamsStatus status;
	double movementUnits[6];
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