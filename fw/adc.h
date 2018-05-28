#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

void adc_init();
int16_t adc_read_single(uint8_t channel);

#endif
