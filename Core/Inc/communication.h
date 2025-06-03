/*
 * communication.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"

typedef enum {
	PWM = 0, DSHOT = 1
} CommunicationType;

typedef struct {
	CommunicationType Type;
	TIM_HandleTypeDef *Timer;
	uint32_t ChannelRise;
	uint32_t ChannelFall;
} CommunicationHandler;

void Communication_TimerInit(CommunicationType commType, TIM_HandleTypeDef *htim, uint32_t channelRise,
		uint32_t channelFall);
void Communication_SetThrottle(void);
void Communication_InputCaptureHandler(TIM_HandleTypeDef *htim);

#endif /* INC_COMMUNICATION_H_ */
