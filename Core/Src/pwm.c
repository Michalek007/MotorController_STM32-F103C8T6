/*
 * pwm.c
 *
 *  Created on: Jun 2, 2025
 *      Author: Michał
 */
#include "pwm.h"
#include "tim.h"

volatile uint16_t pwmDutyCycle = 50;
volatile uint16_t pwmFrequency = 1000;

static volatile uint8_t inputDutyCycle;
static volatile float inputFreq;

void PWM_UpdateDutyCycle(uint8_t targetDutyCycle) {
	uint16_t targetCompareValue = (targetDutyCycle * htim2.Instance->ARR) / 100;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, targetCompareValue);
}

void PWM_UpdateFrequency(uint16_t targetFrequency) {
	uint16_t targetARR = SystemCoreClock / (htim2.Instance->PSC + 1) / targetFrequency - 1;
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

//	__HAL_TIM_SET_PRESCALER(&htim2, newPrescaler);
	__HAL_TIM_SET_AUTORELOAD(&htim2, targetARR);

	__HAL_TIM_SET_COUNTER(&htim2, 0); // optional: reset counter
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (targetARR + 1) / 2);
	PWM_UpdateDutyCycle(pwmDutyCycle);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void PWM_InputCaptureInit(TIM_HandleTypeDef *tim, uint32_t channel_rise, uint32_t channel__fall) {

}

inline void PWM_InputCaptureHandler(TIM_HandleTypeDef *htim) {
	static uint32_t period = 0;
	static uint32_t highTime = 0;
	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			highTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
		inputDutyCycle = (uint8_t) ((((float) highTime / (float) period) * 100.0f) + 0.5f);
		inputFreq = (float) (SystemCoreClock / (htim->Instance->PSC + 1) / (float) (period + 1));
	}
}

uint8_t PWM_GetInputDutyCycle(void) {
	return inputDutyCycle;
}

float PWM_GetInputFreq(void) {
	return inputFreq;
}
