#include "uart_comm.h"
#include "display_manager.h"

extern UART_HandleTypeDef huart1;
extern const Setting mySetting;
extern int8_t control_current_value[16];
extern uint8_t selected_index;
extern char str[32];

void send_byte(uint8_t byte, const char* axis, uint16_t lcd_y) {

    HAL_UART_Transmit(&huart1, &byte, 1, 100);


/*  //lcd message
    char normal_str[20], error_str[20];
    if (HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY) == HAL_OK) {
        sprintf(normal_str, "%5s:done", axis);
        LCD_DrawString(16, lcd_y, normal_str);
    } else {
        sprintf(error_str, "%5s:error", axis);
        LCD_DrawString(16, lcd_y, error_str);
    }
*/

}

void receive_control_values(void) {
    uint8_t byte_to_send = 0xA0;
    HAL_UART_Transmit(&huart1, &byte_to_send, 1, 100);
}
