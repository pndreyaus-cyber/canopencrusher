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

struct SDOParams{
	ParamsStatus status;
	uint8_t nodeId; // 7 bits!
	uint8_t dataLen;
	uint16_t index;
	uint8_t subindex;
	uint32_t data;
	uint8_t dataArray[4]; // 4-byte array converted from data
};

#endif