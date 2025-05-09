#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "main.h"
#include "setting.h"
#include "car_map.h"
#include "car_vector.h"

void display_disconnected(void);
void handle_page_toggle(uint8_t *last_page_toggle);
void process_control_mode(void);
void uint8_to_binary(uint8_t value, char* buffer);
void clear_cursor_column(void);
void redraw_cursor_column(void);
void redraw_setting_range_column(void);
void clear_setting_range_column(void);
void byte_to_binary(uint8_t byte, char* binary_str);
void draw_car_information(uint8_t *data);
void draw_control_page(uint8_t *data, uint32_t len);
void draw_updated_setting(uint8_t index, int8_t value);
void display_arrow_inmap(uint8_t dir, uint8_t speed);
void clear_arrow_inmap(void);
void redraw_map(void);
#endif /* DISPLAY_MANAGER_H */
