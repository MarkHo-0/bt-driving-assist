#include "uart_handler.h"
#include "display_manager.h"
#include "setting.h"
#include "lcd.h"
#include "car_map.h"
#include "car_vector.h"
#include <string.h>  // For memcpy
#include <stdio.h>   // For sprintf

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern const Setting mySetting;
extern int8_t control_current_value[16];
extern char str[32];
extern volatile uint8_t control_page_toggle;
extern uint8_t global_var1[11];
extern uint8_t global_var2[7];
extern int16_t car_x_pos;
extern int16_t car_y_pos;
extern volatile uint8_t drawmap_interrupt_flag;

#define BUFFER_SIZE 256

uint8_t dma_rx_buffer[BUFFER_SIZE];
volatile uint32_t head = 0;  // Updated by DMA/interrupt
uint32_t tail = 0;           // Updated when data is consumed

void start_uart_dma(void) {
    HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) {
    if (drawmap_interrupt_flag==1) return;
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // Update head to the current DMA position
        head = BUFFER_SIZE - hdma_usart1_rx.Instance->CNDTR;
    }
}

/** Get the number of bytes available in the circular buffer */
static uint32_t get_bytes_available(void) {
    return (head - tail + BUFFER_SIZE) % BUFFER_SIZE;
}

/** Extract one byte from the buffer and advance the tail */
static uint8_t get_byte_from_buffer(void) {
    uint8_t byte = dma_rx_buffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    return byte;
}

/** Peek at a byte in the buffer without advancing the tail */
static uint8_t peek_byte(uint32_t offset) {
    return dma_rx_buffer[(tail + offset) % BUFFER_SIZE];
}

/** Process complete packets from the buffer */
void process_packets(void) {
    while (get_bytes_available() >= 1) {
        // Peek at the first byte to determine the expected packet length
        uint8_t first_byte = peek_byte(0);
        uint8_t header = (first_byte >> 6) & 0x03;
        uint32_t expected_length;

        // Determine expected packet length based on header
        if (header == 0b10) {
            expected_length = 2 * mySetting.num_setting; // e.g., 12 bytes if num_setting = 6
        } else if (header == 0b11) {
            expected_length = 2;
        } else if (header == 0b00) {
            expected_length = 11;
        } else if (header == 0b01) {
            expected_length = 7;
        } else {
            // Invalid header, discard the byte and continue
            tail = (tail + 1) % BUFFER_SIZE;
            continue;
            
        }

        // Check if we have enough bytes for a complete packet
        if (get_bytes_available() >= expected_length) {
            // Extract the complete packet
            uint8_t packet[expected_length];
            for (uint32_t i = 0; i < expected_length; i++) {
                packet[i] = get_byte_from_buffer();
            }
            // Process the packet
            process_received_data(packet, expected_length);
        } else {
            // Not enough bytes yet, wait for more data
            break;
        }
    }
}

void process_received_data(uint8_t *data, uint32_t len) {
    if (len == 0) return;
    uint8_t first_byte = data[0];
    uint8_t header = (first_byte >> 6) & 0x03;

    if (header == 0b10 && len == 2 * mySetting.num_setting) {
        if (control_page_toggle == 1) { // Config mode only
            draw_control_page(data, len);
        }
    } else if (header == 0b11 && len == 2) {
        if (control_page_toggle == 1) { // Config mode only
            uint8_t index = first_byte & 0x0F;
            int8_t value = (int8_t)data[1];
            draw_updated_setting(index, value);
        }
    } else if (header == 0b00 && len == 11) {
        memcpy(global_var1, data, 11);
        if (control_page_toggle == 0) { // Car mode only
            
            Vec2i8 chunkCoord = {data[1], data[2]};
            uint64_t chunkdata = 0;
            for (int i = 3; i < 11; i++) {
                chunkdata = (chunkdata << 8) | (uint64_t)data[i];
            }
            setMapChunk(chunkCoord, chunkdata);
        }
    } else if (header == 0b01 && len == 7) {
        memcpy(global_var2, data, 7);
        if (control_page_toggle == 0) { // Car mode only
            clear_arrow_inmap();
            
        }
    }
}