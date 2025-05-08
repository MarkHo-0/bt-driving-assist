#include "car_move.h"

static uint32_t lastControlTime = 0;
volatile Vec2i8 carControl = {0, 0}; // x: forward/backward, y: left/right

extern TIM_HandleTypeDef htim1;
#define SPEED_TIMER (&htim1)
#define SPEED_TIMER_CHANNEL TIM_CHANNEL_3

extern TIM_HandleTypeDef htim3;
#define STEERING_TIMER (&htim3)
#define STEERING_TIMER_CHANNEL TIM_CHANNEL_1

HAL_StatusTypeDef initMoveControl() {
	HAL_StatusTypeDef status = HAL_TIM_PWM_Start(SPEED_TIMER, SPEED_TIMER_CHANNEL);
	if (status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(STEERING_TIMER, STEERING_TIMER_CHANNEL);
	if (status != HAL_OK) return status;

	return HAL_OK;
}

void handle_forward_backward(float MAX_MOVEMENT_SPEED_RATIO) {
	HAL_GPIO_WritePin(Speed_Motor_Ctrl_1_GPIO_Port, Speed_Motor_Ctrl_1_Pin, carControl.y >= 0 ? GPIO_PIN_SET: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Speed_Motor_Ctrl_2_GPIO_Port, Speed_Motor_Ctrl_2_Pin, carControl.y <= 0 ? GPIO_PIN_SET: GPIO_PIN_RESET);

	float maxSpeed = MAX_MOVEMENT_SPEED_RATIO * 99.0f;
	float speedRatio = (float)carControl.y / 32.0f;
	if (speedRatio < 0) speedRatio = -speedRatio;
	int8_t dutyCycle = (int8_t)(speedRatio * maxSpeed);
	__HAL_TIM_SET_COMPARE(SPEED_TIMER, SPEED_TIMER_CHANNEL, dutyCycle);
}

void handle_left_right(uint8_t MAX_STEERING_ANGLE) {
	int min_pulse_us = 500.0f;
	int max_pulse_us = 2500.0f;
	float center_pulse = (min_pulse_us + max_pulse_us) / 2.0f;
	float pulse_per_deg = (max_pulse_us - min_pulse_us) / 180.0f;

	float angle = -((float)carControl.x / 31.0f) * MAX_STEERING_ANGLE;
	float pulse_us = center_pulse + angle * pulse_per_deg;
	__HAL_TIM_SET_COMPARE(STEERING_TIMER, STEERING_TIMER_CHANNEL, (uint16_t)pulse_us);
}

void resetMoveControl() {
    carControl = (Vec2i8){0, 0};
}

void setSpeed(int8_t speed) {
    carControl.y = speed;
    lastControlTime = HAL_GetTick();
}

void setSteering(int8_t dir) {
    carControl.x = dir;
    lastControlTime = HAL_GetTick();
}

void updateMovementControl(float MAX_MOVEMENT_SPEED_RATIO , uint8_t MAX_STEERING_ANGLE, uint16_t autoResetTime) {
    if (HAL_GetTick() - lastControlTime > autoResetTime) resetMoveControl();
    handle_forward_backward(MAX_MOVEMENT_SPEED_RATIO);
    handle_left_right(MAX_STEERING_ANGLE);
}
