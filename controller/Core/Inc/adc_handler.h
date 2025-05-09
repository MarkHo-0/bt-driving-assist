#ifndef ADC_HANDLER_H
#define ADC_HANDLER_H

#include "main.h"
#include "setting.h"

void process_adc_and_transmit(void);
void encode_adc_to_byte(uint32_t adc_value, uint8_t *byte, uint8_t axis);

#endif /* ADC_HANDLER_H */
