#include "display_manager.h"
#include "adc_handler.h"
#include "uart_comm.h"
#include  "lcd.h"
#include "car_map.h"
#include "car_vector.h"
#include <math.h>
#include <stdio.h>   // For sprintf
#include <string.h>  // For memcpy


#define MAP_CENTER_X 112
#define MAP_CENTER_Y 192
#define CAR_TOPLEFT_X 112
#define CAR_TOPLEFT_Y 192
#define DISPLAY_CENTER_X 120    // Center x-coordinate of the map display area
#define DISPLAY_CENTER_Y 200    // Center y-coordinate of the map display area (80 + 240/2)
#define DISPLAY_WIDTH 240       // Width of the map display area
#define DISPLAY_HEIGHT 240      // Height of the map display area
#define CHUNK_PIXEL_DIM 16      // Number of pixels per chunk dimension (adjust as per your system)
#define MAP_MIN_X 0
#define MAP_MAX_X 240
#define MAP_MIN_Y 80
#define MAP_MAX_Y 320
#define ARROWHEAD_LENGTH 10 // Length of the arrowhead lines in pixels
#define ARROWHEAD_ANGLE 45  // Angle in degrees for the arrowhead lines

extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart1;
extern const Setting mySetting;
extern uint8_t selected_index;
extern int8_t control_current_value[16];
extern volatile uint8_t control_page_toggle;
extern volatile int32_t encoderPosCount; //to do
extern char str[32];
extern int16_t car_x_pos;
extern int16_t car_y_pos;
extern int8_t scaling;
extern volatile uint8_t drawmap_interrupt_flag;
float end_x=0;
float end_y=0;
float arrowhead1_x=0;
float arrowhead1_y=0;
float arrowhead2_x=0;
float arrowhead2_y=0;


void display_disconnected(void) {
    LCD_Clear(0, 0, 240, 320, BACKGROUND);
    LCD_DrawString(16, 16, "disconnected");
}

void handle_page_toggle(uint8_t *last_page_toggle) {
    if (control_page_toggle != *last_page_toggle) {
        LCD_Clear(0, 0, 240, 320, BACKGROUND);
        if (control_page_toggle == 0) {
            if (*last_page_toggle==1)
            redraw_map();
        } else {
            LCD_DrawString(80, 0, "Config Mode");
            receive_control_values();
        }
        *last_page_toggle = control_page_toggle;
    }
}

void process_control_mode(void) {
    static uint32_t last_movement_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t adc_y;

    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1000);
    adc_y = HAL_ADC_GetValue(&hadc2);

    if (current_time - last_movement_time >= 300) {
        if (adc_y < 1900) {
            selected_index = (selected_index == 0) ? mySetting.num_setting - 1 : selected_index - 1;
            clear_setting_range_column();
            clear_cursor_column();
            redraw_cursor_column();
            redraw_setting_range_column();
            last_movement_time = current_time;

        } else if (adc_y > 2200) {


            selected_index = (selected_index + 1) % mySetting.num_setting;
            clear_setting_range_column();
            clear_cursor_column();
            redraw_cursor_column();
            redraw_setting_range_column();
            last_movement_time = current_time;


            
        }
    }
}

void uint8_to_binary(uint8_t value, char* buffer) {
    for (int i = 7; i >= 0; --i) {
        buffer[7 - i] = (value & (1 << i)) ? '1' : '0';
    }
    buffer[8] = '\0';
}

void clear_cursor_column(void) {
    uint16_t start_y = 32;
    uint16_t end_y = 32 + (mySetting.num_setting - 1) * 24;
    uint16_t cursor_width = 8;
    LCD_Clear(8, start_y, cursor_width, end_y - start_y + 24, BACKGROUND);
}

void redraw_cursor_column(void) {
    for (uint8_t i = 0; i < mySetting.num_setting; i++) {
        if (i == selected_index) {
            LCD_DrawChar(8, 32 + i * 24, '>');
        } else {
            LCD_DrawChar(8, 32 + i * 24, ' ');
        }
    }
}

// Redraws the range information (min, max, step) below the selected setting
void redraw_setting_range_column(void) {
    sprintf(str, "min:%d max:%d step:%d", 
            mySetting.min[selected_index], 
            mySetting.max[selected_index], 
            mySetting.step[selected_index]);
    char str_3[15];
    char str_2[10];
    sprintf(str_2,"Range:");
    sprintf(str_3,"Unit:%s",mySetting.setting_unit[selected_index]);
    LCD_DrawString(16,240,str_2);        
    LCD_DrawString(16, 256, str);
    LCD_DrawString(16, 270, str_3);
}

// Clears the range information for a specific previous index
void clear_setting_range_column(void) {
    LCD_Clear(16, 256, 224, 32, BACKGROUND); // Clear a 224x16 area (adjust width as needed)
}


void byte_to_binary(uint8_t byte, char* binary_str) {
    for (int i = 7; i >= 0; --i) {
        binary_str[7 - i] = (byte & (1 << i)) ? '1' : '0';
    }
    binary_str[8] = '\0';
}

void draw_control_page(uint8_t *data, uint32_t len) {
    if (len != 2 * mySetting.num_setting) return; // Basic length check

    for (uint8_t i = 0; i < mySetting.num_setting; i++) {
        uint8_t index_byte = data[2 * i];
        uint8_t value_byte = data[2 * i + 1];
        uint8_t index = index_byte & 0x0F;
        int8_t value = (int8_t)value_byte;
        if (index < 16) {
            control_current_value[index] = value;
        }
    }

    // Clear the area for settings display
    LCD_Clear(0, 32, 240, 24 * mySetting.num_setting, BACKGROUND);

    // Display all settings
    for (uint8_t i = 0; i < mySetting.num_setting; i++) {
        sprintf(str, "%18s: %d", mySetting.setting_names[i], control_current_value[i]);
        LCD_DrawString(16, 32 + i * 24, str);
    }
    redraw_cursor_column();
    redraw_setting_range_column();
}

void draw_updated_setting(uint8_t index, int8_t value) {
    if (index >= mySetting.num_setting) return; // Index out of range

    // Clamp the value within min and max limits
    if (value < mySetting.min[index]) value = mySetting.min[index];
    else if (value > mySetting.max[index]) value = mySetting.max[index];

    control_current_value[index] = value;

    // Clear only the area for this specific setting
    LCD_Clear(16, 32 + index * 24, 224, 16, BACKGROUND);

    // Redraw the updated setting
    sprintf(str, "%18s: %d", mySetting.setting_names[index], control_current_value[index]);
    LCD_DrawString(16, 32 + index * 24, str);
}



void draw_car_information(uint8_t *data) {
    // Extract direction byte (assuming it's data[1])
    uint8_t dir = data[1];
    
    // Convert dir to clockwise angle from north
    float phi= ((float)dir / 255.0) * 360.0; // Counter-clockwise from east
   //float phi = 360-theta; // Clockwise from east
   // phi = phi -180;
   // if (phi < 0) phi += 360.0;
    int degree = (int)(phi + 0.5);
    
    // Extract speed byte (assuming it's data[2])
    uint8_t speed = data[2];
    
    // Extract position changes from data[3] to data[6]
    int16_t x_change = (int16_t)((data[3] << 8) | data[4]);
    int16_t y_change = (int16_t)((data[5] << 8) | data[6]);
    
    // Clear larger display area to fit all info
    LCD_Clear(104, 16, 24, 16, BACKGROUND);
    LCD_Clear(80, 32, 48, 16, BACKGROUND);
    LCD_Clear(80, 48, 48, 16, BACKGROUND);
    LCD_Clear(80, 64, 48, 16, BACKGROUND);
    
    char zoom_str[32];
    sprintf(zoom_str, "Zoom:  %1dX",scaling);
    LCD_DrawString(16, 0, zoom_str);
    // Display direction
    char dir_str[32];
    sprintf(dir_str, "Direction: %3d deg", degree);
    LCD_DrawString(16, 16, dir_str);

    // Display speed with one decimal place
    char speed_str[32];
    sprintf(speed_str, "speed:  %6d cm/s", speed);
    LCD_DrawString(16, 32, speed_str);
    
    // Display x position change
    char x_str[32];
    sprintf(x_str, "X pos:  %6d cm", x_change);
    LCD_DrawString(16, 48, x_str);
    
    // Display y position change
    char y_str[32];
    sprintf(y_str, "Y pos:  %6d cm", y_change);
    LCD_DrawString(16, 64, y_str);
}
void clear_arrow_inmap(){
    LCD_DrawLine(MAP_CENTER_X, MAP_CENTER_Y, (uint16_t)end_x, (uint16_t)end_y, BACKGROUND); // Blue color
    LCD_DrawLine((uint16_t)end_x, (uint16_t)end_y, (uint16_t)arrowhead1_x, (uint16_t)arrowhead1_y, BACKGROUND); // Blue color
    LCD_DrawLine((uint16_t)end_x, (uint16_t)end_y, (uint16_t)arrowhead2_x, (uint16_t)arrowhead2_y, BACKGROUND); // Blue color

}

void display_arrow_inmap(uint8_t dir, uint8_t speed) {
    // Calculate angle (0-255 maps to 0°-360°, counter-clockwise from east)
    float theta = ((float)dir / 255.0) * 360.0;
    float base_length = 20.0; // Base arrow length in pixels
    float length = base_length * scaling; // Scale arrow length
    float arrowhead_length = 5.0 * scaling; // Scale arrowhead length

    // Convert to radians
    float theta_rad = theta * (M_PI / 180.0);

    // Calculate the end point of the main line
    end_x = MAP_CENTER_X + length * cos(theta_rad);
    end_y = MAP_CENTER_Y - length * sin(theta_rad); // Subtract because y increases downwards

    // Clamp main line end coordinates to map boundaries
    if (end_x < MAP_MIN_X) end_x = MAP_MIN_X;
    if (end_x > MAP_MAX_X) end_x = MAP_MAX_X;
    if (end_y < MAP_MIN_Y) end_y = MAP_MIN_Y;
    if (end_y > MAP_MAX_Y) end_y = MAP_MAX_Y;

    // Draw the main line
    LCD_DrawLine(MAP_CENTER_X, MAP_CENTER_Y, (uint16_t)end_x, (uint16_t)end_y, 0x001F); // Blue color

    // Calculate the two arrowhead points
    float arrowhead_angle_rad = ARROWHEAD_ANGLE * (M_PI / 180.0);

    // First arrowhead line: 45 degrees clockwise
    arrowhead1_x = end_x - arrowhead_length * cos(theta_rad + arrowhead_angle_rad);
    arrowhead1_y = end_y + arrowhead_length * sin(theta_rad + arrowhead_angle_rad);

    // Second arrowhead line: 45 degrees counterclockwise
    arrowhead2_x = end_x - arrowhead_length * cos(theta_rad - arrowhead_angle_rad);
    arrowhead2_y = end_y + arrowhead_length * sin(theta_rad - arrowhead_angle_rad);

    // Clamp arrowhead coordinates to map boundaries
    if (arrowhead1_x < MAP_MIN_X) arrowhead1_x = MAP_MIN_X;
    if (arrowhead1_x > MAP_MAX_X) arrowhead1_x = MAP_MAX_X;
    if (arrowhead1_y < MAP_MIN_Y) arrowhead1_y = MAP_MIN_Y;
    if (arrowhead1_y > MAP_MAX_Y) arrowhead1_y = MAP_MAX_Y;

    if (arrowhead2_x < MAP_MIN_X) arrowhead2_x = MAP_MIN_X;
    if (arrowhead2_x > MAP_MAX_X) arrowhead2_x = MAP_MAX_X;
    if (arrowhead2_y < MAP_MIN_Y) arrowhead2_y = MAP_MIN_Y;
    if (arrowhead2_y > MAP_MAX_Y) arrowhead2_y = MAP_MAX_Y;

    // Draw the arrowhead lines
    LCD_DrawLine((uint16_t)end_x, (uint16_t)end_y, (uint16_t)arrowhead1_x, (uint16_t)arrowhead1_y, 0x001F); // Blue color
    LCD_DrawLine((uint16_t)end_x, (uint16_t)end_y, (uint16_t)arrowhead2_x, (uint16_t)arrowhead2_y, 0x001F); // Blue color
}


void redraw_map() {
    // Calculate the offset based on scaling

    int offset = DISPLAY_WIDTH / (2 * scaling);
    
    // Calculate the range of map pixels to draw
    int16_t min_mx = car_x_pos - offset;
    int16_t max_mx = car_x_pos + offset - 1;
    int16_t min_my = car_y_pos - offset;
    int16_t max_my = car_y_pos + offset - 1;

    // Iterate over each map pixel in the visible range
    for (int16_t my = min_my; my <= max_my; my++) {
        for (int16_t mx = min_mx; mx <= max_mx; mx++) {
            // Get the pixel value from the map
            uint8_t pixel = getMapPixel((Vec2i16){mx, my});
            uint16_t color = (pixel == 1) ? BLACK : BACKGROUND;
            
            // Calculate the base LCD coordinates for this map pixel
            int16_t lcd_x_base = DISPLAY_CENTER_X + scaling * (mx - car_x_pos);
            int16_t lcd_y_base = DISPLAY_CENTER_Y + scaling * (my - car_y_pos);
            
            // Draw the scaling x scaling block
            for (int dy = 0; dy < scaling; dy++) {
                for (int dx = 0; dx < scaling; dx++) {
                    int16_t lcd_x = lcd_x_base + dx;
                    int16_t lcd_y = lcd_y_base + dy;
                    // Ensure the pixel is within the map display area
                    if (lcd_x >= 0 && lcd_x < DISPLAY_WIDTH && lcd_y >= MAP_MIN_Y && lcd_y < MAP_MIN_Y + DISPLAY_HEIGHT) {
                        LCD_DrawDot(lcd_x, lcd_y, color);
                    }
                }
            }
        }
    }
   
}