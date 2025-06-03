/*
 * pwm.h
 *
 *  Created on: Jun 2, 2025
 *      Author: Michał
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"

void PWM_UpdateDutyCycle(uint8_t targetDutyCycle);
void PWM_UpdateFrequency(uint16_t targetFrequency);

extern volatile uint16_t pwmDutyCycle;
extern volatile uint16_t pwmFrequency;

void PWM_InputCaptureHandler(TIM_HandleTypeDef *htim);
uint8_t PWM_GetInputDutyCycle(void);
float PWM_GetInputFreq(void);

#endif /* INC_PWM_H_ */
