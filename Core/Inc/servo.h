/*
 * servo.h
 *
 *  Created on: Feb 18, 2026
 *      Author: jondbaker
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

// everything here operates on pulse widths (us)
typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	uint16_t min;
	uint16_t max;
	uint16_t pos;
} Servo;

Servo servo_init(
	TIM_HandleTypeDef *timer,
	uint32_t channel,
	uint8_t min,
	uint8_t max
);
void servo_reset(volatile Servo *servo);
void servo_rotate(volatile Servo *servo, uint8_t angle);
uint8_t servo_angle(volatile Servo *servo);

#endif /* INC_SERVO_H_ */
