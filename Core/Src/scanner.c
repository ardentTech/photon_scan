#include "scanner.h"
#include <stdio.h>

#define STEPS_PER_SEQUENCE 16
#define DEGREES_PER_STEP   12

typedef enum {
	PAN_FORWARD,
	PAN_REVERSE,
	TILT_FORWARD,
	TILT_REVERSE,
} Step;

static void step(volatile Scanner *scanner, Step step);

Step sequence[STEPS_PER_SEQUENCE] = {
		//PAN_FORWARD, TILT_FORWARD,
		PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD,
		PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, TILT_FORWARD,
};
volatile Step *next_step = NULL;

Scanner scanner_init(volatile LdrQuad *ldrquad, volatile Servo *pan, volatile Servo *tilt) {
	Scanner scanner = {
			.ldrquad = ldrquad,
			.pan = pan,
			.state = INIT,
			.tilt = tilt
	};
	//	servo_reset(scanner.pan);
	//	servo_reset(scanner.tilt);
	servo_rotate(scanner.pan, 0);
	servo_rotate(scanner.tilt, 0);
	return scanner;
}

void scanner_start(volatile Scanner *scanner) {
	// TODO need to smoothly move to 0s
	//	servo_rotate(scanner->pan, 0);
	//	servo_rotate(scanner->tilt, 0);
	scanner->state = BUSY;
	next_step = sequence;
	scanner_step(scanner);
}

void scanner_step(volatile Scanner *scanner) {
	printf("scanner_step\r\n");
	// TODO handle next_step == NULL
	step(scanner, *next_step);
	ldrquad_read(scanner->ldrquad);
	next_step++;
	if (next_step >= &sequence[STEPS_PER_SEQUENCE]) {
		printf("overflow so done\r\n");
		next_step = NULL;
		scanner->state = DONE;
	}
}

static void step(volatile Scanner *scanner, Step step) {
	printf("step: %d\r\n", step);
	switch (step) {
	case PAN_FORWARD:
		servo_rotate(scanner->pan, servo_angle(scanner->pan) + DEGREES_PER_STEP);
		break;
	case PAN_REVERSE:
		servo_rotate(scanner->pan, servo_angle(scanner->pan) - DEGREES_PER_STEP);
		break;
	case TILT_FORWARD:
		servo_rotate(scanner->tilt, servo_angle(scanner->tilt) + DEGREES_PER_STEP);
		break;
	case TILT_REVERSE:
		servo_rotate(scanner->tilt, servo_angle(scanner->tilt) - DEGREES_PER_STEP);
		break;
	}
}
