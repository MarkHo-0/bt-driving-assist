
#ifndef SRC_CAR_MOVE_H_
#define SRC_CAR_MOVE_H_

#include "stdint.h"
#include "car_vector.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "car_speaker.h"

HAL_StatusTypeDef initMoveControl();

void resetMoveControl();

void setSpeed(int8_t speed);

void setSteering(int8_t dir);

void updateMovementControl(float MAX_MOVEMENT_SPEED_RATIO, uint8_t MAX_STEERING_ANGLE, uint16_t autoResetTime);

HAL_StatusTypeDef qmc5883lReadHeading(float *heading);

float getCurrentHeadingByCache();

#endif /* SRC_CAR_MOVE_H_ */
