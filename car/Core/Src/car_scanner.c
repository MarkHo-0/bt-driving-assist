#include "car_scanner.h"


extern TIM_HandleTypeDef htim3;
#define SCANNER_MOTOR_TIMER (&htim3)
#define SCANNER_MOTOR_CHANNEL TIM_CHANNEL_2 

extern TIM_HandleTypeDef htim2;
#define SCANNER_HCSR04_TIMER (&htim2)
#define SCANNER_HCSR04_CHANNEL TIM_CHANNEL_3

extern I2C_HandleTypeDef hi2c2;

#define HMC5883L_I2C_ADDRESS 0x1E // HMC5883L I2C 地址

#define SYSTEM_DEG_OFFSET 90
#define MY_ABS(x) ((x) < 0 ? -(x) : (x))
#define CALABRATE_SAMPLES 500 // 校正樣本數

static volatile uint32_t echo_start = 0;
static volatile uint32_t echo_end = 0;
static volatile uint8_t is_waiting_rising = 1;

static Vec3i16 hmc_bias = {0.0f, 0.0f, 0.0f}; // 偏差值
static Vec3f hmc_scale = {1.0f, 1.0f, 1.0f}; // 比例值
static float scanner_angle_offset_deg = 0.0f; // 角度偏差值

volatile ScannerState scannerState = SCANNER_IDLE;
volatile ScannerReading scannerReading = {0.0f, 0.0f};
volatile int16_t currentScanerAngle = 0; // 當前掃描器角度


HAL_StatusTypeDef initScanner() {
    //	Configure HMC5883L
	//    uint8_t hmc5883l_config[] = {
	//        0x00, // Configuration Register A
	//        0x7C, // 設定為 75 Hz + 8 samples per measurement
	//        0x01, // Configuration Register B
	//        0x60, // 設定為 ±2.5 Gauss (平衡解析度與範圍)
	//        0x02, // Mode Register
	//        0x00  // Continuous measurement mode
	//    };

    //for (int i = 0; i < sizeof(hmc5883l_config); i += 2) {
    //    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, HMC5883L_I2C_ADDRESS << 1, hmc5883l_config[i], I2C_MEMADD_SIZE_8BIT, &hmc5883l_config[i + 1], 1, 200);
    //    if (status != HAL_OK) return status;
    //}

    HAL_TIM_PWM_Start(SCANNER_MOTOR_TIMER, SCANNER_MOTOR_CHANNEL);
    HAL_TIM_IC_Start_IT(SCANNER_HCSR04_TIMER, SCANNER_HCSR04_CHANNEL);
    HAL_TIM_Base_Start_IT(SCANNER_HCSR04_TIMER);

    return HAL_OK;
}

// -90(最左) ~ 90(最右)度
void rotateScanner(int16_t deg) {
    currentScanerAngle = deg;
    deg += SYSTEM_DEG_OFFSET; // 轉換成 0~180 度
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;

    uint16_t pulse_width = 500 + ((uint32_t)deg * 2000) / 180;
    __HAL_TIM_SET_COMPARE(SCANNER_MOTOR_TIMER, SCANNER_MOTOR_CHANNEL, pulse_width);
}

void swingScanner(uint8_t cycle, uint8_t range, uint8_t offset) {
    if (cycle == 0) return;
    uint32_t t = HAL_GetTick() % (cycle * 1000);

    float half_cycle = (cycle * 1000) / 2.0f;
    float angle = (1.0f - MY_ABS((float)t - half_cycle) / half_cycle) * 2 * range - range;

    rotateScanner((int16_t)(angle + offset));
}

void hcsr04SendWave() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    is_waiting_rising = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(SCANNER_HCSR04_TIMER, SCANNER_HCSR04_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
}

HAL_StatusTypeDef hmc5883lReadRaw3D(Vec3i16 *reading) {
    uint8_t data[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, 0x1E << 1, 0x03, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    if (status != HAL_OK) return status;

    // 傳感器朝下，z 軸反轉
    reading->x = (int16_t)((data[0] << 8) | data[1]);
    reading->z = -(int16_t)((data[2] << 8) | data[3]);
    reading->y = (int16_t)((data[4] << 8) | data[5]);

    return HAL_OK;
}

HAL_StatusTypeDef hmc5883lReadHeading() {
    Vec3i16 rawReading = {0, 0, 0};
    HAL_StatusTypeDef status = hmc5883lReadRaw3D(&rawReading);
    if (status != HAL_OK) return status;

    // 修正讀數，將原始讀數減去偏差值，然後乘上比例值
    Vec3f correctedReading = (Vec3f){
        (rawReading.x - hmc_bias.x) * hmc_scale.x,
        (rawReading.y - hmc_bias.y) * hmc_scale.y,
        (rawReading.z - hmc_bias.z) * hmc_scale.z
    };

    // 計算方向角度，使用 atan2 函數計算 y/x 的反正切值，然後轉換成度數
     float original_heading = atan2f(correctedReading.y, correctedReading.x) * 57.295779513f;
     if (original_heading < 0.0f) original_heading += 360.0f;
     if (original_heading >= 360.0f) original_heading -= 360.0f;
 
     // 套用角度偏差值
     float adjusted_heading = original_heading - scanner_angle_offset_deg;
     adjusted_heading = fmodf(adjusted_heading, 360.0f);
     if (adjusted_heading < 0.0f) adjusted_heading += 360.0f;
 
     // 將調整後的角度設置到 scannerReading 結構體中
     scannerReading.heading = adjusted_heading;
 
     return HAL_OK;
}

void startScan() {
    if (scannerState != SCANNER_IDLE) return;

    hcsr04SendWave();
    scannerState = SCANNER_SENT;
}

HAL_StatusTypeDef hmc5883lCalibrateOnce() {
    static int collectedSamples = 0;
    static Vec3i16 min, max;

    if (collectedSamples == 0) {
        min = (Vec3i16){INT16_MAX, INT16_MAX, INT16_MAX};
        max = (Vec3i16){INT16_MIN, INT16_MIN, INT16_MIN};
    }

    Vec3i16 rawReading = {0, 0, 0};
    HAL_StatusTypeDef status = hmc5883lReadRaw3D(&rawReading);
    if (status != HAL_OK) return HAL_BUSY; // 再次測量

    if (rawReading.x < min.x) min.x = rawReading.x;
    if (rawReading.y < min.y) min.y = rawReading.y;
    if (rawReading.z < min.z) min.z = rawReading.z;

    if (rawReading.x > max.x) max.x = rawReading.x;
    if (rawReading.y > max.y) max.y = rawReading.y;
    if (rawReading.z > max.z) max.z = rawReading.z;

    if (++collectedSamples < 1000) return HAL_BUSY;

    hmc_bias = vec3i16_from_vec3f(vec3i16_scale_f(vec3i16_add(min, max), 0.5f));
    hmc_scale = vec3f_conj(vec3i16_scale_f(vec3i16_sub(max, min), 0.5f));

    collectedSamples = 0;
    return HAL_OK;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (is_waiting_rising) {
    	__HAL_TIM_SET_COUNTER(SCANNER_HCSR04_TIMER, 0);
        is_waiting_rising = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(SCANNER_HCSR04_TIMER, SCANNER_HCSR04_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
        scannerState = SCANNER_WAITING;
    } else {
    	uint32_t delta = HAL_TIM_ReadCapturedValue(SCANNER_HCSR04_TIMER, SCANNER_HCSR04_CHANNEL);
        scannerReading.distance = delta / 58.0f;

        scannerState = SCANNER_DONE;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (scannerState == SCANNER_WAITING) {
        scannerState = SCANNER_FAILED;
    }
}

void setScannerAngleOffset(float offsetDeg) {
    scanner_angle_offset_deg = offsetDeg;
}

Vec2i16 getObstaclePosition(Vec2i16 viwerPosition){
    Vec2i16 offset = {
        (int16_t)(scannerReading.distance * cosf(scannerReading.heading * M_PI / 180.0f)),
        (int16_t)(scannerReading.distance * sinf(scannerReading.heading * M_PI / 180.0f))
    };
    return vec2i16_add(viwerPosition, offset);
}
