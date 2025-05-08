#include "car_motion_sensors.h"

// --- 全局變數 ---
Vec3f carVelocity     = {0.0f, 0.0f, 0.0f};
Vec3f carPosition     = {0.0f, 0.0f, 0.0f};
Vec3f carRotation_rad = {0.0f, 0.0f, 0.0f};  // x=Roll, y=Pitch, z=Yaw (radians)

volatile Vec3i16 qmc_bias = {0, 0, 0}; // 磁力計偏置
volatile Vec3f qmc_scale = {1.0f, 1.0f, 1.0f}; // 磁力計縮放因子
volatile Vec3i16 mpu_acccel_bias = {0, 0, 0}; // 加速度計偏置
volatile Vec3i16 mpu_gyro_bias = {0, 0, 0}; // 陀螺儀偏置

// --- 常數 ---

// MPU6050
#define MPU6050_ADDR               (0x68 << 1)
#define MPU6050_ACCEL_XOUT_H_REG   0x3B
#define MPU6050_ACCEL_SCALE_4G     8192.0f  // LSB/g
#define MPU6050_GYRO_SCALE_500DPS  65.5f    // LSB/deg/s

// QMC5883L
#define QMC5883L_ADDR              (0x0D << 1)
#define QMC5883L_STATUS_REG        0x06
#define QMC5883L_DATAX_L_REG       0x00
#define QMC5883L_SCALE_8G          3000.0f   // LSB/Gauss (approximate, check datasheet for exact)

// 物理常量
#define GRAVITY                   9.80665f   // 重力加速度 (m/s²)
#define DEG_TO_RAD                0.0174533f // 角度轉弧度轉換因子
#define RAD_TO_DEG                57.2958f   // 弧度轉角度轉換因子
#define MAG_DECLINATION           0.0f       // 磁偏角 (弧度，需根據所在地調整)

// 零值檢測閾值
#define ACCEL_NOISE_THRESHOLD     0.9f      // 加速度計噪聲閾值
#define GYRO_NOISE_THRESHOLD      0.5f       // 陀螺儀噪聲閾值
#define VELOCITY_ZERO_THRESHOLD   0.01f      // 速度零值閾值

// 互補濾波器係數
#define ACCEL_WEIGHT              0.02f      // 加速度計權重
#define MAG_WEIGHT                0.1f      // 磁力計權重
#define GYRO_WEIGHT               (1.0f - MAG_WEIGHT)  // 陀螺儀權重

/**
 * @brief 使用歐拉角旋轉向量 (Z-Y-X順序)
 *
 * 將輸入向量v按照指定的歐拉角(roll-x, pitch-y, yaw-z)進行旋轉，
 * 旋轉順序為：先繞Z軸(yaw)，再繞Y軸(pitch)，最後繞X軸(roll)。
 * 採用右手坐標系，正旋轉方向為逆時針。
 *
 * @param v 待旋轉的原始向量
 * @param euler_rad 歐拉角(弧度制)，分量分別為(x-roll, y-pitch, z-yaw)
 * @return Vec3f 旋轉後的新向量
 */
Vec3f rotate_vector_by_euler(Vec3f v, Vec3f euler_rad) {
    const float cr = cosf(euler_rad.x);  // cos(roll)
    const float sr = sinf(euler_rad.x);  // sin(roll)
    const float cp = cosf(euler_rad.y);  // cos(pitch)
    const float sp = sinf(euler_rad.y);  // sin(pitch)
    const float cy = cosf(euler_rad.z);  // cos(yaw)
    const float sy = sinf(euler_rad.z);  // sin(yaw)

    return (Vec3f){
        .x = v.x * (cy*cp) + v.y * (cy*sp*sr - sy*cr) + v.z * (cy*sp*cr + sy*sr),
        .y = v.x * (sy*cp) + v.y * (sy*sp*sr + cy*cr) + v.z * (sy*sp*cr - cy*sr),
        .z = v.x * (-sp)    + v.y * (cp*sr)            + v.z * (cp*cr)
    };
}

HAL_StatusTypeDef initMotionSensors() {
    HAL_StatusTypeDef status;
    uint8_t data[2];

    //wait for I2C ready
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        HAL_Delay(10);  // Wait for I2C to be ready
    }

    // --- Initialize MPU6050 ---

    // Check if MPU6050 is connected
    const uint16_t WHO_AM_I_ADDR = 0x75;  // WHO_AM_I register address
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_ADDR, I2C_MEMADD_SIZE_8BIT, data, 1, 200);
    if (status != HAL_OK || data[0] != 0x68) return status;  // Check WHO_AM_I register value

    // Reset MPU6050 and configure settings in one go
    uint8_t mpu6050_config[] = {
        // --- Basic Setup ---
        0x6B, 0x80,          // 1. Reset MPU6050
        0x6B, 0x01,          // 2. Wake up, Clock Source = Gyro X PLL
        0x19,  9,            // 3. Sample Rate = 1kHz / (1 + 9) = 100Hz
        0x1A,  2,            // 4. DLPF config = 2 (Accel BW=94Hz, Gyro BW=98Hz)
        0x1B, (1 << 3),      // 5. Gyro Range = +/- 500 dps (0x08)
        0x1C, (1 << 3),      // 6. Accel Range = +/- 4g (0x08)
    
        // --- Motion Detection Settings ---
		0x21, 10,          // ZRMOT_THR：threshold，越細越敏感（試 10）
		0x22, 20,          // ZRMOT_DUR：大約 20 * 1ms = 20ms 無動作才觸發
    
        // --- Interrupt Enable & Pin Config ---
        0x69, 0x40,        // MOT_DETECT_CTRL: ZMOT_EN = 1（bit 6）
        0x38, 0x20,        // INT_ENABLE: Zero Motion Interrupt enable（bit 5）
		0x37, 0x02         // 10. INT Pin Config: Active High, Push-Pull, 50us Pulse, Bypass Enabled.
    };

    for (int i = 0; i < sizeof(mpu6050_config); i += 2) {
        data[0] = mpu6050_config[i];
        data[1] = mpu6050_config[i + 1];
        status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 200);
        if (status != HAL_OK) return status;
        HAL_Delay(50);
    }

    // --- Initialize QMC5883L ---

    // Reset QMC5883L and configure settings in one go
    uint8_t qmc5883l_config[] = {
        0x0A, 0x80,  // Soft Reset
        0x09, (0 << 6) | (1 << 4) | (1 << 2) | 1,  // Configure OSR=512, Range=8G, ODR=50Hz, Mode=Continuous
        0x0B, 0x01   // Set/Reset Period (default)
    };

    for (int i = 0; i < sizeof(qmc5883l_config); i += 2) {
        data[0] = qmc5883l_config[i];
        data[1] = qmc5883l_config[i + 1];
        status = HAL_I2C_Master_Transmit(&hi2c1, QMC5883L_ADDR, data, 2, 200);
        if (status != HAL_OK) return status;
        HAL_Delay(50);
    }

    return HAL_OK;
}

HAL_StatusTypeDef qmc5883lReadRaw3D(Vec3i16 *reading) {
    uint8_t data[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR, QMC5883L_DATAX_L_REG, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    if (status != HAL_OK) return status;

    // 傳感器朝下，z 軸反轉
    reading->x = (int16_t)((data[1] << 8) | data[0]);
    reading->y = (int16_t)((data[3] << 8) | data[2]);
    reading->z = -(int16_t)((data[5] << 8) | data[4]);

    return HAL_OK;
}

HAL_StatusTypeDef mpu6050ReadRaw3D(Vec3i16 *accel_reading, Vec3i16 *gyro_reading) {
    uint8_t data[14]; 
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, data, 14, 100);
    if (status != HAL_OK) return status;

    // 傳感器朝下，z 軸反轉
    accel_reading->x = (int16_t)((data[0] << 8) | data[1]);
    accel_reading->y = (int16_t)((data[2] << 8) | data[3]);
    accel_reading->z = -(int16_t)((data[4] << 8) | data[5]);
    gyro_reading->x = (int16_t)((data[8] << 8) | data[9]);
    gyro_reading->y = (int16_t)((data[10] << 8) | data[11]);
    gyro_reading->z = -(int16_t)((data[12] << 8) | data[13]);

    return HAL_OK;
}
HAL_StatusTypeDef qmc5883lReadHeading(float *heading) {
    Vec3i16 rawReading = {0, 0, 0};
    HAL_StatusTypeDef status = qmc5883lReadRaw3D(&rawReading);
    if (status != HAL_OK) return status;

    // 修正讀數，將原始讀數減去偏差值，然後乘上比例值
    Vec3f correctedReading = (Vec3f){
        (rawReading.x - qmc_bias.x) * qmc_scale.x,
        (rawReading.y - qmc_bias.y) * qmc_scale.y,
        (rawReading.z - qmc_bias.z) * qmc_scale.z
    };

    // 計算方向角度，使用 atan2 函數計算 y/x 的反正切值，然後轉換成度數
    *heading = atan2f(correctedReading.y, correctedReading.x) * RAD_TO_DEG;
    if (*heading < 0.0f) *heading += 360.0f;
    return HAL_OK;
}

HAL_StatusTypeDef updateMotionData(float dt) {
    // 讀取傳感器數據
    Vec3i16 mag_reading, accel_reading, gyro_reading;
    if (qmc5883lReadRaw3D(&mag_reading) != HAL_OK) return HAL_ERROR;
    if (mpu6050ReadRaw3D(&accel_reading, &gyro_reading) != HAL_OK) return HAL_ERROR;

    // 校正數據
    Vec3f mag_corrected = vec3f_mul(vec3f_from_vec3i16(vec3i16_sub(mag_reading, qmc_bias)), qmc_scale);
    Vec3f accel_corrected = vec3i16_scale_f(vec3i16_sub(accel_reading, mpu_acccel_bias), (1.0f / MPU6050_ACCEL_SCALE_4G) * GRAVITY);
    //Vec3f gyro_corrected = vec3i16_scale_f(vec3i16_sub(gyro_reading, mpu_gyro_bias), (1.0f / MPU6050_GYRO_SCALE_500DPS) * DEG_TO_RAD);

    // 透過磁場計算角度
    Vec3f mag_rot = (Vec3f){
        .x = atan2f(mag_corrected.z, sqrtf(mag_corrected.x * mag_corrected.x + mag_corrected.y * mag_corrected.y)),
        .y = atan2f(-mag_corrected.x, mag_corrected.y),
        .z = atan2f(mag_corrected.y, mag_corrected.x) + MAG_DECLINATION
    };

    // 確保在 [0, 2π) 範圍內
    if (mag_rot.x < 0) mag_rot.x += 2 * M_PI;
    if (mag_rot.z < 0) mag_rot.z += 2 * M_PI;
    if (mag_rot.y < 0) mag_rot.y += 2 * M_PI;

    // 透過陀螺儀計算角度變化
    //Vec3f deltaRot = vec3f_scale(gyro_corrected, dt);

    // 使用互補濾波器更新姿態
    //carRotation_rad = vec3f_add(vec3f_scale(vec3f_add(carRotation_rad, deltaRot), GYRO_WEIGHT), vec3f_scale(mag_rot, MAG_WEIGHT));
    
    // 直接使用磁場計算的角度
    carRotation_rad = mag_rot;
    
    // 將加速度轉換到世界座標系
    Vec3f world_accel = rotate_vector_by_euler(accel_corrected, carRotation_rad);
    

    // 過濾噪聲，只在加速度大於閾值時更新位置和速度
    float accel_mag = vec3f_mag(world_accel);
    if (accel_mag > ACCEL_NOISE_THRESHOLD) {
        carVelocity = vec3f_add(carVelocity, vec3f_scale(world_accel, dt));
    }

    float velocity_mag = vec3f_mag(carVelocity);
    if (velocity_mag > VELOCITY_ZERO_THRESHOLD) {
        carPosition = vec3f_add(carPosition, vec3f_scale(carVelocity, dt));
    }

    return HAL_OK;
}

HAL_StatusTypeDef qmc5883lCalibrateOnce() {
    static int collectedSamples = 0;
    static Vec3i16 min, max;

    // 初始化最小值和最大值
    if (collectedSamples == 0) {
        min = (Vec3i16){INT16_MAX, INT16_MAX, INT16_MAX};
        max = (Vec3i16){INT16_MIN, INT16_MIN, INT16_MIN};
    }

    // 讀取原始數據
    Vec3i16 rawReading;
    if (qmc5883lReadRaw3D(&rawReading) != HAL_OK) return HAL_BUSY;

    // 更新最小值和最大值
    if (rawReading.x < min.x) min.x = rawReading.x;
    if (rawReading.y < min.y) min.y = rawReading.y;
    if (rawReading.z < min.z) min.z = rawReading.z;
    
    if (rawReading.x > max.x) max.x = rawReading.x;
    if (rawReading.y > max.y) max.y = rawReading.y;
    if (rawReading.z > max.z) max.z = rawReading.z;

    // 等待收集足夠的樣本數
    collectedSamples++;
    if (collectedSamples < 1000) return HAL_BUSY; 

    // 計算偏差值和縮放因子
    qmc_bias = vec3i16_from_vec3f(vec3i16_scale_f(vec3i16_add(min, max), 0.5f));
    qmc_scale = vec3f_conj(vec3i16_scale_f(vec3i16_sub(max, min), 0.5f));

    // 重置樣本計數器
    collectedSamples = 0; 
    return HAL_OK;
}

HAL_StatusTypeDef mpu6050CalibrateOnce() {
    static int collectedSamples = 0;
    static Vec3i64 accel_sum, gyro_sum;

    // 初始化累加器
    if (collectedSamples == 0) {
        accel_sum = (Vec3i64){0, 0, 0};
        gyro_sum = (Vec3i64){0, 0, 0};
    }
    
    // 讀取原始數據
    Vec3i16 accel_reading, gyro_reading;
    if (mpu6050ReadRaw3D(&accel_reading, &gyro_reading) != HAL_OK) return HAL_BUSY;

    // 加速度計和陀螺儀數據累加
    accel_sum = vec3i64_add(accel_sum, vec3i64_from_vec3i16(accel_reading));
    gyro_sum = vec3i64_add(gyro_sum, vec3i64_from_vec3i16(gyro_reading));

    // 等待收集足夠的樣本數
    collectedSamples++;
    if (collectedSamples < 200) return HAL_BUSY;

    // 除以樣本數，計算平均值
    mpu_acccel_bias = vec3i16_from_vec3f(vec3i64_scale_f(accel_sum, 1.0f / collectedSamples));
    mpu_gyro_bias = vec3i16_from_vec3f(vec3i64_scale_f(gyro_sum, 1.0f / collectedSamples));
    
    // 重置樣本計數器
    collectedSamples = 0; 
    return HAL_OK;
}

HAL_StatusTypeDef mayUpdateMotionData() {
    static uint32_t lastUpdateTime = 0;
    uint32_t timeDiff = HAL_GetTick() - lastUpdateTime;

    if (timeDiff >= 200) {  // 每200ms更新一次
        HAL_StatusTypeDef status = updateMotionData(timeDiff / 1000.0f);  // Convert ms to seconds
        lastUpdateTime = HAL_GetTick();
        return status;
    }

    return HAL_BUSY;
}

void resetMotionData() {
    carVelocity = (Vec3f){0.0f, 0.0f, 0.0f};
    carPosition = (Vec3f){0.0f, 0.0f, 0.0f};
    carRotation_rad = (Vec3f){0.0f, 0.0f, 0.0f};
}

float getCurrentHeadingByCache() {
    // 計算方向角度，使用 atan2 函數計算 y/x 的反正切值，然後轉換成度數
	float heading = fmodf(carRotation_rad.z * 57.2958f, 360.0f);
	if (heading < 0) heading += 360.0f;
    return heading;
}

void generateMotionPackage(uint8_t package[7]) {
    // --- Heading (radians to degrees, adjust reference, scale to 0-255) ---
    // Ensure yaw_deg is in [0, 360) range relative to North
    float yaw_deg = fmodf(carRotation_rad.z * RAD_TO_DEG, 360.0f);
    if (yaw_deg < 0) yaw_deg += 360.0f;

    // --- Speed (m/s to cm/s, float to uint8_t) ---
    float speed_mm_s = vec3f_mag(carVelocity) * 100.0f;
    if (speed_mm_s > 255.0f) speed_mm_s = 255.0f;

    // --- Position (m to cm, float to int16_t, split into MSB and LSB) ---
    int16_t pos_x_cm = (int16_t)(carPosition.x * 100.0f + (carPosition.x > 0 ? 0.5f : -0.5f));
    int16_t pos_y_cm = (int16_t)(carPosition.y * 100.0f + (carPosition.y > 0 ? 0.5f : -0.5f));

    package[0] = 0x40;                          // Binary 01000000
    package[1] = (uint8_t)((yaw_deg / 360.0f) * 255.0f);  // Scale [0, 360) to [0, 255]
    package[2] = (uint8_t)speed_mm_s;
    package[3] = (uint8_t)((pos_x_cm >> 8) & 0xFF);  // X MSB
    package[4] = (uint8_t)(pos_x_cm & 0xFF);         // X LSB
    package[5] = (uint8_t)((pos_y_cm >> 8) & 0xFF);  // Y MSB
    package[6] = (uint8_t)(pos_y_cm & 0xFF);         // Y LSB
}
