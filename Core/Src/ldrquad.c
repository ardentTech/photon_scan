/*
 * ldrquad.c
 *
 *  Created on: Feb 19, 2026
 *      Author: jondbaker
 */
#include "ldrquad.h"

LdrQuad ldrquad_init(ADC_HandleTypeDef *adc, void (*Error_Handler)(void)) {
	LdrQuad ldrquad = {
		.adc = adc,
	};

	// TODO might need to explicitly init .buffer

	if (HAL_ADC_Start_DMA(ldrquad.adc, (uint32_t *)ldrquad.buffer, ADC_CHANNELS) != HAL_OK) {
		Error_Handler();
	}
	return ldrquad;
}

void ldrquad_read(volatile LdrQuad *ldrquad) {
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
