#include "callback.h"
#include "main.h"
#include "display_manager.h"

extern volatile int32_t encoderPosCount;
extern volatile uint32_t last_pb13_interrupt_time;
extern volatile uint32_t last_pb15_interrupt_time;
extern volatile uint32_t last_pb12_interrupt_time;
extern volatile uint8_t control_page_toggle;
extern uint8_t selected_index;
extern UART_HandleTypeDef huart1;
extern int8_t scaling;

void handle_car_map(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pb13_interrupt_time > 40) {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET){
                scaling++;
                if (scaling > 5) scaling = 5;
                redraw_map();
            }
            else{
                scaling--;
                if (scaling < 1) scaling = 1;
                redraw_map();
            }
            last_pb13_interrupt_time = current_time;
        }
    }
}

void handle_control_setting(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pb13_interrupt_time > 100) {
            last_pb13_interrupt_time = current_time;
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) {
                                uint8_t byte_to_send = 0x80 | (selected_index & 0x0F);
                HAL_UART_Transmit(&huart1, &byte_to_send, 1, 100);
            } else {
                                uint8_t byte_to_send = 0x90 | (selected_index & 0x0F);
                HAL_UART_Transmit(&huart1, &byte_to_send, 1, 100);
            }
        }
    }
}

void handle_page_flag_toggle(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_15) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pb15_interrupt_time > 50) {
            control_page_toggle = !control_page_toggle;
            last_pb15_interrupt_time = current_time;
        }
    }
}

void handle_speaker_action(uint16_t GPIO_Pin) { //PULL DOWN OR PULL UP
    if (GPIO_Pin == GPIO_PIN_12 && control_page_toggle == 0) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pb12_interrupt_time > 50) {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) { // PULL DOWN
                uint8_t byte_to_send = 0xE0;
                HAL_UART_Transmit(&huart1, &byte_to_send, 1, 100);
                
            /* lcd message
                if (HAL_UART_Transmit(&huart1, &byte_to_send, 1, 100) == HAL_OK) {
                    char n0_str[20];
                    sprintf(n0_str, "%13s", "SPEAKER:SENT");
                    LCD_DrawString(16, 48, n0_str);
                } else {
                    char n1_str[20];
                    sprintf(n1_str, "%13s", "SPEAKER:NO");
                    LCD_DrawString(16, 48, n1_str);
                }
            */
            } else { //PULL UP

            /* lcd message
                char n3_str[20];
                sprintf(n3_str, "%13s", "Released");
                LCD_DrawString(16, 48, n3_str);
            */

            }
            last_pb12_interrupt_time = current_time;
        }
    }
}
