#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "car_speaker.h"

#define MELODY_SIZE 3  // 每段 melody 固定三個 tone
#define Speaker_Channel TIM_CHANNEL_2 // PWM 控制的通道
extern TIM_HandleTypeDef htim4; // PWM 控制的定時器

static const Tone melodies[MELODY_COUNT][MELODY_SIZE] = {
    [MELODY_STARTUP]      = {{523, 100}, {659, 100}, {784, 300}},
    [MELODY_HORN]         = {{600, 300}, {0, 100}, {600, 200}},
    [MELODY_CONNECT_OK]   = {{800, 100}, {1000, 100}, {1200, 150}},
    [MELODY_CONNECT_FAIL] = {{1000, 150}, {800, 150}, {600, 200}},
    [MELODY_SHORT_BEEP]   = {{1000, 50}, {0, 10}, {0, 10}},
    [MELODY_LONG_BEEP]    = {{1000, 200}, {0, 100}, {0, 100}}
};

static SpeakerState speaker = {MELODY_NONE, 0, 0};

void setSpeakerFrequency(uint16_t freqHz) {
    if (freqHz == 0) {  // 停 PWM
        HAL_TIM_PWM_Stop(&htim4, Speaker_Channel);
        return;
    }

    uint32_t arr = 1000000 / freqHz;
    if (arr > 0xFFFF) arr = 0xFFFF;

    __HAL_TIM_SET_AUTORELOAD(&htim4, arr - 1);
    __HAL_TIM_SET_COMPARE(&htim4, Speaker_Channel, arr / 2);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_PWM_Start(&htim4, Speaker_Channel); // 開 PWM
}

void playMelody(MelodyType type) {
    if (type >= MELODY_COUNT) return;
    speaker = (SpeakerState){type, 0, HAL_GetTick()};
    setSpeakerFrequency(melodies[type][0].frequency);
}

void muteSpeaker() {
    setSpeakerFrequency(0);
    speaker = (SpeakerState){MELODY_NONE, 0, 0};
}

void updateSpeaker() {
    if (speaker.currentMelody == MELODY_NONE) return;

    uint32_t nowMs = HAL_GetTick();    
    const Tone* currentTone = &melodies[speaker.currentMelody][speaker.currentTone];

    if (nowMs - speaker.lastChangeTime >= currentTone->duration) {
        speaker.currentTone++;

        if (speaker.currentTone >= MELODY_SIZE) {
            // 播完
            muteSpeaker();
        } else {
            // 播下一個Tone
            speaker.lastChangeTime = nowMs;
            setSpeakerFrequency(melodies[speaker.currentMelody][speaker.currentTone].frequency);
        }
    }
}