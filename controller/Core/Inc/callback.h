#ifndef CALLBACK_H
#define CALLBACK_H

#include <stdint.h>

void handle_car_map(uint16_t GPIO_Pin);
void handle_control_setting(uint16_t GPIO_Pin);
void handle_page_flag_toggle(uint16_t GPIO_Pin);
void handle_speaker_action(uint16_t GPIO_Pin);

#endif // CALLBACK_FUN_H
