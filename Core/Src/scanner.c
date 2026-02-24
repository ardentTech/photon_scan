#include "scanner.h"

#define STEPS_PER_SEQUENCE 65
#define DEGREES_PER_STEP   18

typedef enum {
	PAN_FORWARD,
	PAN_REVERSE,
	TILT_FORWARD,
	TILT_REVERSE,
} Step;

static void step(volatile Scanner *scanner, Step step);

Step sequence[STEPS_PER_SEQUENCE] = {
		PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD,
		PAN_FORWARD, PAN_FORWARD, TILT_FORWARD, // 108
		PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE,
		PAN_REVERSE, PAN_REVERSE, TILT_FORWARD, // 126
		PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD,
		PAN_FORWARD, PAN_FORWARD, TILT_FORWARD, // 144
		PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE,
		PAN_REVERSE, PAN_REVERSE, TILT_FORWARD, // 162
		PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD, PAN_FORWARD,
		PAN_FORWARD, PAN_FORWARD, TILT_FORWARD, // 180
		PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE, PAN_REVERSE,
		PAN_REVERSE, PAN_REVERSE
};
volatile Step *next_step = NULL;

void scanner_analyze(volatile Scanner *scanner) {
	LdrQuadReading last_reading = ldrquad_raw_reading(scanner->ldrquad);
	uint16_t avg_reading = ldrquad_avg_reading(&last_reading);
	if (avg_reading < scanner->result.min) {
		scanner->result.min = avg_reading;
		scanner->result.min_pos = (Position) { servo_angle(scanner->pan), servo_angle(scanner->tilt) };
	}
	if (avg_reading > scanner->result.max) {
		scanner->result.max = avg_reading;
		scanner->result.max_pos = (Position) { servo_angle(scanner->pan), servo_angle(scanner->tilt) };
	}
}

Scanner scanner_init(volatile LdrQuad *ldrquad, volatile Servo *pan, volatile Servo *tilt) {
	Scanner scanner = {
			.ldrquad = ldrquad,
			.pan = pan,
			.state = INIT,
			.tilt = tilt,
			.result = (ScanResult) {
				.min = UINT16_MAX,
				.min_pos = { 0, 0 },
				.max = 0,
				.max_pos = { 0, 0 },
			}
	};
	// TODO smooth these out
	servo_rotate(scanner.pan, 0);
	servo_rotate(scanner.tilt, 0);
	return scanner;
}

void scanner_start(volatile Scanner *scanner) {
	scanner->result = (ScanResult) {
		.min = UINT16_MAX,
		.min_pos = { 0, 0 },
		.max = 0,
		.max_pos = { 0, 0 },
	};
	scanner->state = BUSY;
	next_step = sequence;
	scanner_step(scanner);
}

void scanner_step(volatile Scanner *scanner) {
	if (next_step != NULL) {
		step(scanner, *next_step);
		ldrquad_read(scanner->ldrquad);
		next_step++;
		if (next_step >= &sequence[STEPS_PER_SEQUENCE]) {
			next_step = NULL;
			scanner->state = DONE;
		}
	}
}

static void step(volatile Scanner *scanner, Step step) {
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
