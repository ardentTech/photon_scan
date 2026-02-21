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
    volatile uint16_t buffer[ADC_CHANNELS];
} LdrQuad;

typedef struct {
    uint16_t ne;
    uint16_t se;
    uint16_t sw;
    uint16_t nw;
} LdrQuadReading;

LdrQuad ldrquad_init(ADC_HandleTypeDef *adc, void (*Error_Handler)(void));
void ldrquad_read(volatile LdrQuad *ldrquad);

#endif /* INC_LDRQUAD_H_ */
