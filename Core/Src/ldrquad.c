#include "ldrquad.h"

// Starts the ADC conversion.
void ldrquad_read(volatile LdrQuad *ldrquad) {
    HAL_ADC_Start(ldrquad->adc);
}

// Generates a reading from the buffer. This should be called once the ADC conversion is complete.
LdrQuadReading ldrquad_get_reading(volatile LdrQuad *ldrquad) {
    return (LdrQuadReading) {
        .ne = ldrquad->buffer[0],
		.se = ldrquad->buffer[1],
		.sw = ldrquad->buffer[2],
		.nw = ldrquad->buffer[3],
    };
}

// Starts the DMA associated with the ADC.
void ldrquad_start_dma(volatile LdrQuad *ldrquad) {
	HAL_ADC_Start_DMA(ldrquad->adc, (uint32_t *)ldrquad->buffer, ADC_CHANNELS);
}
