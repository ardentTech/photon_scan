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

typedef struct {
	volatile LdrQuad *ldrquad;
	volatile Servo *pan;
	volatile Servo *tilt;
} Scanner;

Scanner scanner_init(volatile LdrQuad *ldrquad, volatile Servo *pan, volatile Servo *tilt);

#endif /* SRC_SCANNER_H_ */
