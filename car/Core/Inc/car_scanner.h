#ifndef SRC_CAR_SCANNER_H_
#define SRC_CAR_SCANNER_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "math.h"
#include <car_vector.h>

// ----- 狀態 State Enum -----
typedef enum {
    SCANNER_IDLE,
    SCANNER_SENT, 
    SCANNER_WAITING,
    SCANNER_FAILED,
    SCANNER_DONE
} ScannerState;

typedef struct {
    float distance;  // 距離 (cm)
    float heading;  // 角度 (degree)
} ScannerReading;

// 初始化函式
HAL_StatusTypeDef initScanner();

// 非阻塞擺動控制
void swingScanner(uint8_t cycle, uint8_t range, uint8_t offset);

// 控制馬達轉到指定角度
void rotateScanner(int16_t deg);

// 開始一次距離掃描 (如果狀態係 IDLE)，會將 Trig Pin 拉高再拉低，然後啟動 Timer 同 Interrupt
void startScan();

// 停止掃描
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

// 每次 Timer 到達時會呼叫這個函式
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// 校準磁力計，每次呼叫代表採集一次樣本，直到返回 HAL_OK 代表校準完成
HAL_StatusTypeDef hmc5883lCalibrateOnce();

// 讀取角度，並將結果存入 scannerReading 結構中
HAL_StatusTypeDef hmc5883lReadHeading();

// 設定掃描器的角度偏差值，這個值會在計算障礙物位置時使用
void setScannerAngleOffset(float offsetDeg);
 
// 在完成掃描後計算，根據當前位置計算障礙物位置
Vec2i16 getObstaclePosition(Vec2i16 carPosition);

extern volatile ScannerState scannerState;
extern volatile ScannerReading scannerReading;
extern volatile int16_t currentScanerAngle; // 當前掃描器角度 (degree)

#endif /* SRC_CAR_SCANNER_H_ */
