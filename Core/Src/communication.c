/*
 * communication.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#include "communication.h"
#include "dshot.h"
#include "usart.h"
//#include "mc_api.h"

#define PWM_BUFFER_SIZE 2
#define DSHOT_BUFFER_SIZE 32
#define DSHOT_US_FACTOR 1

static uint16_t pwmCounterBuffer[PWM_BUFFER_SIZE] = { 0 };
static uint16_t dshotCounterBuffer[DSHOT_BUFFER_SIZE] = { 0 };

static CommunicationHandler commHandler = { 0 };

static volatile uint16_t pwmPulseWidth;
static volatile uint8_t pwmRxFlag = 0;
static volatile uint8_t dshotRxFlag = 0;

void PWM_WriteBuffer(TIM_HandleTypeDef *htim);
void Dshot_WriteBuffer(TIM_HandleTypeDef *htim);

void Communication_TimerInit(CommunicationType commType, TIM_HandleTypeDef *htim, uint32_t channelRise,
		uint32_t channelFall) {
	commHandler.Type = commType;
	commHandler.Timer = htim;
	commHandler.ChannelRise = channelRise;
	commHandler.ChannelFall = channelFall;
	uint32_t prescaler = 0;
	switch (commType) {
	case PWM:
		prescaler = (SystemCoreClock / 1000000) - 1;
		break;
	case DSHOT:
		prescaler = (SystemCoreClock / (DSHOT_US_FACTOR * 1000000)) - 1;
		break;
	default:
		break;
	}
	__HAL_TIM_SET_PRESCALER(htim, prescaler);
	HAL_TIM_IC_Start_IT(htim, channelRise);
	HAL_TIM_IC_Start_IT(htim, channelFall);
}

inline void PWM_WriteBuffer(TIM_HandleTypeDef *htim) {
	static uint8_t head = 0;
	static uint8_t risingEdgeFlag = 0;

	if (!risingEdgeFlag && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		risingEdgeFlag = 1;
		pwmCounterBuffer[0] = HAL_TIM_ReadCapturedValue(htim, commHandler.ChannelRise);
		++head;
	} else if (risingEdgeFlag && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		risingEdgeFlag = 0;
		pwmCounterBuffer[1] = HAL_TIM_ReadCapturedValue(htim, commHandler.ChannelFall);
		++head;
	}
	if (head >= PWM_BUFFER_SIZE) {
		head = 0;
		pwmPulseWidth = pwmCounterBuffer[1] - pwmCounterBuffer[0];
		pwmRxFlag = 1;
	}
}

//inline void PWM_WriteBuffer(TIM_HandleTypeDef *htim) {
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
////		uint32_t period = HAL_TIM_ReadCapturedValue(htim, commHandler.ChannelRise);
//	} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
//		pwmPulseWidth = HAL_TIM_ReadCapturedValue(htim, commHandler.ChannelFall);
//		pwmRxFlag = 1;
//	}
//}

void PWM_SetThrottle(void) {
	static uint16_t lastPulseWidth = 0;
	uint16_t currentPulseWidth = pwmPulseWidth;

	if (lastPulseWidth != pwmPulseWidth) {
		lastPulseWidth = currentPulseWidth;
		UART_Printf("%u\n", pwmPulseWidth);
		if (currentPulseWidth <= 2000 && currentPulseWidth >= 1500) {
//			int16_t targetSpeedUnit = ((int16_t) (currentPulseWidth - 1500) * MAX_APPLICATION_SPEED_UNIT) / 500;
//			MC_ProgramSpeedRampMotor1(targetSpeedUnit, 1);
			UART_Print("Setting motor speed. ");
		}
	}
}

inline void Dshot_WriteBuffer(TIM_HandleTypeDef *htim) {
	static uint8_t head = 0;
	static uint8_t risingEdgeFlag = 0;

	if (dshotRxFlag) {
		return;
	}

	if (!risingEdgeFlag && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		risingEdgeFlag = 1;
		dshotCounterBuffer[head] = HAL_TIM_ReadCapturedValue(htim, commHandler.ChannelRise);
		++head;
	} else if (risingEdgeFlag && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		risingEdgeFlag = 0;
		dshotCounterBuffer[head] = HAL_TIM_ReadCapturedValue(htim, commHandler.ChannelFall);
		++head;
	}
	if (head >= DSHOT_BUFFER_SIZE) {
		head = 0;
		dshotRxFlag = 1;
	}
}

void Dshot_SetThrottle(void) {
	uint16_t pulseWidthUs[DSHOT_BUFFER_SIZE / 2];
	for (uint8_t i = 0; i < DSHOT_BUFFER_SIZE / 2; ++i) {
		pulseWidthUs[i] = (dshotCounterBuffer[2 * i + 1] - dshotCounterBuffer[2 * i]) / DSHOT_US_FACTOR;
	}
	DShotPacket dshotPacket = { 0 };
	DShot_DeserializePulseWidthUs(&dshotPacket, pulseWidthUs, DSHOT_BUFFER_SIZE / 2, DSHOT150);
	UART_Printf("%u\n", dshotPacket.throttle);
	UART_Printf("%u\n", dshotPacket.telemetry);
	UART_Printf("%u\n", dshotPacket.crc);
	if (!DShot_ValidateCrc(&dshotPacket)) {
		return;
	}

	if (dshotPacket.throttle >= 48) {
//		int16_t targetSpeedUnit = ((int16_t) (dshotPacket.throttle - 48) * MAX_APPLICATION_SPEED_UNIT) / 2000;
//		MC_ProgramSpeedRampMotor1(targetSpeedUnit, 1);
		UART_Print("Setting motor speed. ");
	}
}

void Communication_SetThrottle(void) {
	if (commHandler.Type == PWM) {
		if (pwmRxFlag) {
			pwmRxFlag = 0;
			PWM_SetThrottle();
		}
	} else if (commHandler.Type == DSHOT) {
		if (dshotRxFlag) {
			dshotRxFlag = 0;
			Dshot_SetThrottle();
		}
	}
}

inline void Communication_InputCaptureHandler(TIM_HandleTypeDef *htim) {
	if (commHandler.Type == PWM) {
		PWM_WriteBuffer(htim);
	} else if (commHandler.Type == DSHOT) {
		Dshot_WriteBuffer(htim);
	}
}

