/*
 * motor_pwm.h
 *
 *  Created on: Jun 2, 2025
 *      Author: Michał
 */

#ifndef INC_MOTOR_PWM_H_
#define INC_MOTOR_PWM_H_

#include "main.h"

void MotorPwm_Init(TIM_HandleTypeDef *tim, uint32_t channel);

#endif /* INC_MOTOR_PWM_H_ */
