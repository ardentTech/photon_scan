#include "scanner.h"

Scanner scanner_init(LdrQuad *ldrquad, Servo *pan, Servo *tilt) {
	Scanner scanner = {
			.ldrquad = ldrquad,
			.pan = pan,
			.tilt = tilt
	};
	servo_reset(scanner.pan);
	servo_reset(scanner.tilt);
	return scanner;
}
