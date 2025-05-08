#include "car_setting.h"

// --- Settings Definition ---
const Setting settings[TOTAL_SETTINGS_COUNT] = {
    { 1, 5, 1, 3 },       // Maximum Movement Speed
    { 5, 30, 5, 15 },      // Maximum Steering Angle
    { 0, 1, 1, 1 },        // Enable Scan
    { 1, 10, 1, 3 },       // Scan Cycle
    { 10, 90, 5, 45 },     // Scan Range
    { -90, 90, 1, 0 },     // Scan Offset
    { 10, 127, 5, 50},     // Scan Distance
    { 1, 6, 1, 2}          // Safty Distance
};

// --- Global Settings Values ---
int8_t settings_values[TOTAL_SETTINGS_COUNT];

void initDefaultSettings(void) {
    for (int i = 0; i < TOTAL_SETTINGS_COUNT; ++i) {
        settings_values[i] = settings[i].default_value;
    }
}
