/*
 * ldrquad.h
 *
 *  Created on: Feb 19, 2026
 *      Author: jondbaker
 */

#ifndef INC_LDRQUAD_H_
#define INC_LDRQUAD_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_adc.h"

#define ADC_CHANNELS (4UL)

typedef struct {
    ADC_HandleTypeDef *adc;
    uint16_t buffer[ADC_CHANNELS];
} LdrQuad;

typedef struct {
    uint16_t ne;
    uint16_t se;
    uint16_t sw;
    uint16_t nw;
} LdrQuadReading;

void ldrquad_read(volatile LdrQuad *ldrquad);
LdrQuadReading ldrquad_raw_reading(volatile LdrQuad *ldrquad);
uint16_t ldrquad_avg_reading(LdrQuadReading *reading);
void ldrquad_start_dma(volatile LdrQuad *ldrquad);

#endif /* INC_LDRQUAD_H_ */
