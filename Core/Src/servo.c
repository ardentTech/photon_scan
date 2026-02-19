#include "servo.h"
#include "utils.h"

#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define MIN_PULSE 500
#define MAX_PULSE 2500

uint16_t angle_to_pulse(const uint16_t angle);
uint16_t pulse_to_angle(const uint16_t pulse);

// initializes a servo
Servo servo_init(
		TIM_HandleTypeDef *timer,
		const uint32_t channel,
		uint8_t min,
		uint8_t max
) {
	Servo servo = (Servo) {
		.timer = timer,
		.channel = channel,
		.min = angle_to_pulse(min),
		.max = angle_to_pulse(max),
		.pos = 0,
	};
	return servo;
}

uint16_t angle_to_pulse(const uint16_t angle) {
	return map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE);
}

// converts a pulse width to an angle
uint16_t pulse_to_angle(const uint16_t pulse) {
	return map(pulse, MIN_PULSE, MAX_PULSE, MIN_ANGLE, MAX_ANGLE);
}

// returns the current servo position as an angle
uint8_t servo_angle(Servo* servo) {
	return pulse_to_angle(servo->pos);
}

// resets a servo to its neutral position
void servo_reset(Servo* servo) {
	servo_rotate(servo, MAX_ANGLE / 2);
}

// rotates a servo to a specific angle
void servo_rotate(Servo* servo, uint8_t angle) {
	uint16_t new_pos = angle_to_pulse(angle);
	if (new_pos < servo->min) {
		new_pos = servo->min;
	} else if (new_pos > servo->max) {
		new_pos = servo->max;
	}
	__HAL_TIM_SET_COMPARE(servo->timer, servo->channel, new_pos);
	servo->pos = new_pos;
}
