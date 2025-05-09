#include "adc_handler.h"
#include "uart_comm.h"
#include "display_manager.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern char str[32];

void process_adc_and_transmit(void) {
    uint32_t adc_x, adc_y, adc_x0;
    uint8_t x_byte, y_byte;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    HAL_ADC_PollForConversion(&hadc1, 1000);
    adc_x0 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_PollForConversion(&hadc2, 1000);
    adc_y = HAL_ADC_GetValue(&hadc2);
    adc_x = 4095 - adc_x0;
 /* lcd message
    char str_x[20], str_y[20];
    sprintf(str_x, "X:%10d", adc_x);
    LCD_DrawString(16, 16, str_x);
    sprintf(str_y, "Y:%10d", adc_y0);
    LCD_DrawString(16, 32, str_y);
*/
    encode_adc_to_byte(adc_x, &x_byte, 1);
    encode_adc_to_byte(adc_y, &y_byte, 0);
/* lcd message
    uint8_to_binary(x_byte, str);
    LCD_DrawString(16, 96, str);
    uint8_to_binary(y_byte, str);
    LCD_DrawString(16, 112, str);
*/
    if ((x_byte & 0x3F) != 0) {
    	send_byte(x_byte, "X", 128);
    }
    if ((y_byte & 0x3F) != 0) {
    	send_byte(y_byte, "Y", 144);
    }
}

void encode_adc_to_byte(uint32_t adc_value, uint8_t *byte, uint8_t axis) {
    int8_t mag;
    if (adc_value >= 1900 && adc_value <= 2200) {
        mag = 0; // Dead zone
    } else if (adc_value < 1900) {
        mag = -32 + (adc_value * 32) / 1900;
    } else {
        int32_t u = adc_value - 2200;
        mag = 1 + (u * 31) / 1900;
    }
    *byte = (axis << 6) | (mag & 0x3F);
}
