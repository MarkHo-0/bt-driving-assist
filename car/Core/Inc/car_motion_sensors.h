#ifndef SRC_CAR_MOTION_SENSORS_H_
#define SRC_CAR_MOTION_SENSORS_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "car_vector.h"
// --- Struct Definitions ---

// --- Global Variables ---

extern Vec3f carVelocity;       // Car velocity (m/s) - Note: Highly prone to drift
extern Vec3f carPosition;       // Car position (m) - Note: Highly prone to drift
extern Vec3f carRotation_rad;   // Car attitude/rotation (Roll, Pitch, Yaw in radians)

extern I2C_HandleTypeDef hi2c1; // I2C handle for the sensors

// --- Function Prototypes ---

HAL_StatusTypeDef initMotionSensors();

HAL_StatusTypeDef updateMotionData(float dt);

HAL_StatusTypeDef qmc5883lReadHeading(float *heading);

HAL_StatusTypeDef mayUpdateMotionData();

// 校準磁力計，每次呼叫代表採集一次樣本，直到返回 HAL_OK 代表校準完成
HAL_StatusTypeDef qmc5883lCalibrateOnce();

// 校準加速計和陀螺儀，每次呼叫代表採集一次樣本，直到返回 HAL_OK 代表校準完成
HAL_StatusTypeDef mpu6050CalibrateOnce();

void resetMotionData();

void generateMotionPackage(uint8_t package[7]);

#endif /* SRC_CAR_MOTION_SENSORS_H_ */
