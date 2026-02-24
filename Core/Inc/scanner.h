/*
 * scanner.h
 *
 *  Created on: Feb 19, 2026
 *      Author: jondbaker
 */

#ifndef SRC_SCANNER_H_
#define SRC_SCANNER_H_

#include "ldrquad.h"
#include "servo.h"

typedef enum {
	INIT,
	BUSY,
	DONE,
} ScannerState;

typedef struct {
	uint8_t pan;
	uint8_t tilt;
} Position;

typedef struct {
	uint16_t min;
	Position min_pos;
	uint16_t max;
	Position max_pos;
} ScanResult;

typedef struct {
	volatile LdrQuad *ldrquad;
	volatile Servo *pan;
	ScannerState state;
	volatile Servo *tilt;
	ScanResult result;
} Scanner;

Scanner scanner_init(volatile LdrQuad *ldrquad, volatile Servo *pan, volatile Servo *tilt);
void scanner_start(volatile Scanner *scanner);
void scanner_step(volatile Scanner *scanner);
void scanner_analyze(volatile Scanner *scanner);

#endif /* SRC_SCANNER_H_ */
