#ifndef PARAMS_H
#define PARAMS_H
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

struct ReceivedMessage
{
	uint32_t id;
	uint8_t data[8]; // Maximum data length is 8 bytes
	uint8_t len;
};

#endif