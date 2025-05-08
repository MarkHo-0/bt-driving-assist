#ifndef SRC_CARSPEAKER_H_
#define SRC_CARSPEAKER_H_

typedef struct {
    uint16_t frequency;
    uint16_t duration;
} Tone;

typedef enum {
    MELODY_NONE = -1,
    MELODY_STARTUP = 0, 
    MELODY_HORN,
    MELODY_CONNECT_OK,
    MELODY_CONNECT_FAIL,
    MELODY_SHORT_BEEP,
    MELODY_LONG_BEEP,
    MELODY_COUNT
} MelodyType;

typedef struct {
    MelodyType currentMelody;
    uint8_t currentTone;
    uint32_t lastChangeTime;
} SpeakerState;

void setSpeakerFrequency(uint16_t freqHz);
void playMelody(MelodyType type);
void updateSpeaker();

#endif /* SRC_CARSPEAKER_H_ */
