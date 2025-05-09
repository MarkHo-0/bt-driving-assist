#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"
#include "setting.h"

void send_byte(uint8_t byte, const char* axis, uint16_t lcd_y);
void receive_control_values(void);

#endif /* UART_COMM_H */
