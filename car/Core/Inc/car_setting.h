#ifndef CAR_SETTING_H
#define CAR_SETTING_H

#include <stdint.h>
#include "string.h"

#define SETTINGS_FILENAME "settings.dat"

typedef struct {
    int8_t min;
    int8_t max;
    int8_t step;
    int8_t default_value;
} Setting;

typedef enum {
    IDX_MAX_MOVEMENT_SPEED = 0,
    IDX_MAX_STEERING_ANGLE,
    IDX_ENABLE_SCAN,
    IDX_SCAN_CYCLE,
    IDX_SCAN_RANGE,
    IDX_SCAN_OFFSET,
    IDX_SCAN_DISTANCE,
    IDX_SAFETY_DISTANCE,
    TOTAL_SETTINGS_COUNT
} SettingIndex;

extern const Setting settings[TOTAL_SETTINGS_COUNT];
extern int8_t settings_values[TOTAL_SETTINGS_COUNT];

void initDefaultSettings(void);

#endif // CAR_SETTING_H
