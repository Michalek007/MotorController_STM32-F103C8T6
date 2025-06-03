/*
 * motor_pwm.c
 *
 *  Created on: Jun 2, 2025
 *      Author: Michał
 */

#include "motor_pwm.h"
#include "dshot.h"
#include "tim.h"

static PwmHandler pwmHandler;

void MotorPwm_Init(TIM_HandleTypeDef *tim, uint32_t channel) {
	pwmHandler.Tim = tim;
	pwmHandler.Channel = channel;
	uint16_t freqHz = 50;
	uint16_t targetARR = SystemCoreClock / (tim->Instance->PSC + 1) / freqHz - 1;
	__HAL_TIM_SET_AUTORELOAD(tim, targetARR);
	__HAL_TIM_SET_COMPARE(tim, channel, 0);
	HAL_TIM_PWM_Start(tim, channel);
}

/*
 * Thorttle 0-999 value
 * 0 means 1ms duty cycle (full left)
 * 500 means 1.5 ms duty cycle (stop)
 * 1000 means 2 ms duty cyccle (full right)
 */
void MotorPwm_SetThrottle(uint16_t throttle) {
	uint16_t targetCompareValue = throttle + 1000;
	__HAL_TIM_SET_COMPARE(pwmHandler.Tim, pwmHandler.Channel, targetCompareValue);
}
