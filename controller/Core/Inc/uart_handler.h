#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "main.h"

#define DMA_BUFFER_SIZE 256

extern uint8_t received_message[256];
extern uint32_t received_length;
extern volatile uint8_t message_ready;

void start_uart_dma(void);
void process_received_data(uint8_t *data, uint32_t len);

#endif
