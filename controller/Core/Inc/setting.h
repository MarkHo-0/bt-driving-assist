#ifndef SETTING_H
#define SETTING_H

typedef struct {
    int8_t min[16];
    int8_t max[16];
    uint8_t step[16];
    uint8_t num_setting;
    const char* setting_names[16];
    const char* setting_unit[16];
} Setting;

#endif
