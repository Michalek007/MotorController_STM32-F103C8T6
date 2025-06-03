/*
 * dshot.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "main.h"

#define DSHOT_BIT_COUNT 16

typedef enum {
	DSHOT150, DSHOT300, DSHOT600, DSHOT1200
} DShotType;

typedef struct {
	TIM_HandleTypeDef *Tim;
	uint32_t Channel;
} PwmHandler;

void DShot_TimerInit(DShotType dshotType, TIM_HandleTypeDef *tim, uint32_t channel);
void DShot_SendPacket(uint16_t throttle, uint8_t telemetry);

/**
 * @brief Represents a 16-bit DShot packet.
 *
 * This struct defines a DShot packet, used in the DShot digital
 * ESC (Electronic Speed Controller) communication protocol for RC applications.
 *
 * The DShot packet format is as follows:
 * - 11 bits for the throttle value (0–2047, 0-47 are reserved for special commands)
 * - 1 bit for the telemetry request flag (0 = no telemetry, 1 = request telemetry)
 * - 4 bits for a CRC checksum (XOR-based)
 *
 */
typedef struct {
	uint16_t throttle;
	uint8_t telemetry;
	uint8_t crc;
} DShotPacket;

void DShot_Deserialize(DShotPacket *dshotPacket, uint8_t *data, uint8_t size);
void DShot_DeserializePulseWidthUs(DShotPacket *dshotPacket, uint16_t *data, uint8_t size, DShotType dshotType);
uint8_t DShot_ValidateCrc(DShotPacket *dshotPacket);

#endif /* INC_DSHOT_H_ */
