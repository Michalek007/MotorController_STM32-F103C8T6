/*
 * dshot.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */
#include "dshot.h"
#include "assert.h"

static uint16_t dshotDmaBuffer[DSHOT_BIT_COUNT];
static PwmHandler pwmHandler = { 0 };

void DShot_TimerInit(DShotType dshotType, TIM_HandleTypeDef *tim, uint32_t channel) {
	pwmHandler.Tim = tim;
	pwmHandler.Channel = channel;
	uint32_t prescaler = 0;
	switch (dshotType) {
	case DSHOT150:
		prescaler = (SystemCoreClock / 10000 / 30) - 1;
//		prescaler = (SystemCoreClock / 150000 / 30) - 1;
		break;
	case DSHOT300:
		prescaler = (SystemCoreClock / 300000 / 30) - 1;
		break;
	case DSHOT600:
		prescaler = (SystemCoreClock / 600000 / 30) - 1;
		break;
	case DSHOT1200:
		prescaler = (SystemCoreClock / 1200000 / 30) - 1;
		break;
	default:
		break;
	}
	__HAL_TIM_SET_PRESCALER(tim, prescaler);
	__HAL_TIM_SET_AUTORELOAD(tim, 30);
	__HAL_TIM_SET_COMPARE(tim, channel, 0);
}

uint16_t DShot_BuildPacket(uint16_t throttle, uint8_t telemetry) {
	throttle &= 0x7FF; // throttle is 11 bit (2047 max)
	throttle <<= 1; // shift left to make space for telemetry and crc
	throttle |= (0x01 & telemetry);
	uint16_t crc = 0;
	for (uint8_t i = 0; i < 3; i++) {
		crc ^= (throttle >> (i * 4)) & 0xF;
	}
	return (throttle << 4) | (crc & 0xF);
}

static void DShot_PrepareDmaBuffer(uint16_t dshotPacket) {
	for (uint8_t i = 0; i < DSHOT_BIT_COUNT; i++) {
		if (dshotPacket & (1 << (15 - i))) {
			dshotDmaBuffer[i] = (uint16_t) (0.75f * (float) pwmHandler.Tim->Instance->ARR);  // logic 1: ~75% of ARR
		} else {
			dshotDmaBuffer[i] = (uint16_t) (0.375f * (float) pwmHandler.Tim->Instance->ARR); // logic 0: ~37.5% of ARR
		}
	}
}

void DShot_SendPacket(uint16_t throttle, uint8_t telemetry) {
	uint16_t packet = DShot_BuildPacket(throttle, telemetry);
	DShot_PrepareDmaBuffer(packet);
	HAL_TIM_PWM_Start_DMA(pwmHandler.Tim, pwmHandler.Channel, (uint32_t*) dshotDmaBuffer, DSHOT_BIT_COUNT);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == pwmHandler.Tim->Instance) {
		HAL_TIM_PWM_Stop_DMA(pwmHandler.Tim, pwmHandler.Channel);
	}
}

void DShot_Deserialize(DShotPacket *dshotPacket, uint8_t *data, uint8_t size) {
	if (size != DSHOT_BIT_COUNT) {
		return;
	}
	uint16_t dshotData = 0;
	for (uint8_t i = 0; i < size; i++) {
		if (data[i] >= 60) {
			dshotData |= (1 << (15 - i));
		}
	}
	dshotPacket->throttle = (dshotData >> 5) & 0x7FF;
	dshotPacket->telemetry = (dshotData >> 4) & 0x01;
	dshotPacket->crc = dshotData & 0xF;
}

void DShot_DeserializePulseWidthUs(DShotPacket *dshotPacket, uint16_t *data, uint8_t size, DShotType dshotType) {
	if (size != DSHOT_BIT_COUNT) {
		return;
	}
	uint16_t logicOneThreshold = 0;
	switch (dshotType) {
	case DSHOT150:
		logicOneThreshold = 4;
		break;
	case DSHOT300:
		logicOneThreshold = 2;
		break;
	case DSHOT600:
		logicOneThreshold = 1;
		break;
	case DSHOT1200:
		break;
	default:
		break;
	}
	uint16_t dshotData = 0;
	for (uint8_t i = 0; i < size; i++) {
		if (data[i] >= logicOneThreshold) {
			dshotData |= (1 << (15 - i));
		}
	}
	dshotPacket->throttle = (dshotData >> 5) & 0x7FF;
	dshotPacket->telemetry = (dshotData >> 4) & 0x01;
	dshotPacket->crc = dshotData & 0xF;
}

uint8_t DShot_ValidateCrc(DShotPacket *dshotPacket) {
	uint16_t data = (dshotPacket->throttle << 1) | dshotPacket->telemetry;
	uint16_t crc = 0;
	for (uint8_t i = 0; i < 3; i++) {
		crc ^= (data >> (i * 4)) & 0xF;
	}
	if (crc == dshotPacket->crc) {
		return 1;
	}
	return 0;
}
