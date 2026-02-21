/*
 * ldrquad.c
 *
 *  Created on: Feb 19, 2026
 *      Author: jondbaker
 */
#include "ldrquad.h"
#include <stdbool.h>

volatile bool dma_started = false;

void ldrquad_read(LdrQuad *ldrquad) {
	if (!dma_started) {
		HAL_ADC_Start_DMA(ldrquad->adc, (uint32_t *)ldrquad->buffer, ADC_CHANNELS);
	}
    HAL_ADC_Start(ldrquad->adc);
}

LdrQuadReading ldrquad_get_reading(LdrQuad *ldrquad) {
    return (LdrQuadReading) {
        .ne = ldrquad->buffer[0],
		.se = ldrquad->buffer[1],
		.sw = ldrquad->buffer[2],
		.nw = ldrquad->buffer[3],
    };
}
