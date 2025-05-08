/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "math.h"

#include "car_vector.h"
#include "car_move.h"
#include "car_setting.h"
#include "car_speaker.h"
#include "car_scanner.h"
#include "car_motion_sensors.h"
#include "car_map.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile CalibrationState calibrationState = CALIBRATION_IDLE;
static volatile uint32_t lastSuccessScanTime = 0;
static volatile uint8_t isFinishedInit = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RX_BUFFER_SIZE 10
uint8_t rxBuffer[RX_BUFFER_SIZE];
static volatile uint8_t isConnectedToRemote = 0;
static volatile uint8_t scannerAngleMeasured = 0;

inline static void handle_hc05_state_changed() {
  isConnectedToRemote = HAL_GPIO_ReadPin(HC05_State_GPIO_Port, HC05_State_Pin) == GPIO_PIN_SET;
  playMelody(isConnectedToRemote ? MELODY_CONNECT_OK : MELODY_CONNECT_FAIL);
}

inline static void handle_car_no_motion() {
  carVelocity = (Vec3f){0, 0, 0};
}

inline static void handle_calabration_request() {
  if (calibrationState != CALIBRATION_IDLE) return;
  calibrationState = CALIBRATION_REQUESTED;
}

inline static void handle_scanner_angle_ready() {
	return;
  if (hmc5883lReadHeading() != HAL_OK) return;
  scannerAngleMeasured = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (0 == isFinishedInit) return; // Ignore if not finished init
  if (CALIBRATION_IDLE != calibrationState) return; // Ignore if calibration is in progress
  if (HC05_State_Pin == GPIO_Pin) handle_hc05_state_changed();
  else if (GY87_EXTI_Pin == GPIO_Pin) handle_car_no_motion();
  else if (Calibration_BTN_Pin == GPIO_Pin) handle_calabration_request();
}

void send_all_settings_values() {
	uint8_t packet[TOTAL_SETTINGS_COUNT * 2];

	for (uint8_t i = 0; i < TOTAL_SETTINGS_COUNT; ++i) {
	  packet[i * 2]     = (0b10 << 6) | (i & 0x0F);
	  packet[i * 2 + 1] = (uint8_t)((int8_t)settings_values[i]);
	}

	HAL_UART_Transmit(&huart1, packet, sizeof(packet), 200);
}

void send_update_setting_value(uint8_t setting_id) {
	uint8_t packet[2];
	packet[0] = (0b11 << 6) | (setting_id & 0x0F);
	packet[1] = (uint8_t)settings_values[setting_id];

	HAL_UART_Transmit(&huart1, packet, 2, 200);
}

void handle_setting_change(uint8_t setting_id, int8_t delta) {
  if (setting_id >= TOTAL_SETTINGS_COUNT || delta == 0) return;
  
  int8_t new_value = settings_values[setting_id] + delta * settings[setting_id].step;

  if (new_value >= settings[setting_id].min && new_value <= settings[setting_id].max) {
    settings_values[setting_id] = new_value;
    send_update_setting_value(setting_id);
  }
}

void process_command(uint8_t cmd) {
  // 檢查第 1 個 bit：0 = 移動控制
  if ((cmd & 0x80) == 0x00) {
    // 取出 6-bit 有符號數值
    int8_t ctl_val = cmd & 0x3F; // 取後6位
    if (ctl_val & 0x20) ctl_val |= 0xC0;   // 符號擴展

    // 再用第 2 個 bit 分類：0 = 前後，1 = 左右
    if ((cmd & 0x40) == 0x00) {
      setSpeed(ctl_val);
    } else {
      setSteering(ctl_val);
    }
  }

  // 檢查頭三個 bit 是否為 100 = 設定修改
  else if ((cmd & 0xE0) == 0x80) {
    uint8_t setting_idx = cmd & 0x0F;              // 低 4-bit 為設定編號
    int8_t delta = (cmd & 0x10) ? -1 : +1;         // 第 4-bit 為修改  1 = 減，0 = 加）
    handle_setting_change(setting_idx, delta);
  }

  // 取得所有設定：101xxxxx
  else if ((cmd & 0xE0) == 0xA0) {
    send_all_settings_values();
  }

  // 嚮喇叭：111xxxxx
  else if ((cmd & 0xE0) == 0xE0) {
    playMelody(MELODY_HORN);
  }
}

void onChunkUpdated(Vec2i8 chunk_coord, uint64_t chunkData) {
  uint8_t chunkPackage[11] = {};
  generateChunkPackage(chunk_coord, chunkData, chunkPackage);
  playMelody(MELODY_SHORT_BEEP);
  HAL_UART_Transmit(&huart1, chunkPackage, sizeof(chunkPackage), 200);
}

HAL_StatusTypeDef mayCalibrateOnce() {
  if (CALIBRATION_IDLE == calibrationState) return HAL_OK;

  // 檢查是否有校準請求，如果有播放短音
  if (CALIBRATION_REQUESTED == calibrationState) {
    playMelody(MELODY_SHORT_BEEP);
    calibrationState = CALIBRATION_STATIC;
    return HAL_BUSY;
  }

  // 如果當前狀態是靜止校準，則進行加速計和陀螺儀校準
  if (CALIBRATION_STATIC == calibrationState) {
	  HAL_Delay(10);
    const HAL_StatusTypeDef mpu_status = mpu6050CalibrateOnce();
    if (mpu_status != HAL_OK) return HAL_BUSY;

    playMelody(MELODY_SHORT_BEEP);
    calibrationState = CALIBRATION_MOVING;
    return HAL_BUSY;
  }

  // 如果當前狀態是移動校準，則進行磁力計校準
  if (CALIBRATION_MOVING == calibrationState) {
	  HAL_Delay(10);
    const HAL_StatusTypeDef qmc_status = qmc5883lCalibrateOnce();
    if (qmc_status != HAL_OK) return HAL_BUSY;

    calibrationState = CALIBRATION_DONE;
    return HAL_BUSY;
  }

  // 如果當前狀態是校準完成，則重置狀態
  if (CALIBRATION_DONE == calibrationState) {
    playMelody(MELODY_LONG_BEEP);
    calibrationState = CALIBRATION_IDLE;
    resetMotionData();
    resetMap();
    return HAL_OK;
  }

  // 應該不會到這裡....
  return HAL_ERROR;
}

HAL_StatusTypeDef calibrateAccelGyroOnly() {
  while (1) {
    HAL_StatusTypeDef status = mpu6050CalibrateOnce();
    if (status == HAL_OK) return HAL_OK;
    if (status != HAL_BUSY) return status;
    HAL_Delay(10);
  }
}

void printCarFacingAndSensorFacing() {
  char sText[100] = {};
  int car_facing_deg = getCurrentHeadingByCache();
  int sensor_facing_deg = scannerReading.heading;
  int sensor_distance = scannerReading.distance;
  snprintf(sText, sizeof(sText), "Car facing: %i deg, Scanner facing: %i deg, Distance: %i cm\r\n", car_facing_deg, sensor_facing_deg, sensor_distance);
  HAL_UART_Transmit(&huart1, sText, sizeof(sText),200);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  initDefaultSettings();

  if (initMoveControl() == HAL_OK && 
      initScanner() == HAL_OK && 
      initMotionSensors() == HAL_OK &&
      calibrateAccelGyroOnly() == HAL_OK) {
    playMelody(MELODY_STARTUP);
    isFinishedInit = 1;
  }

  while (1)
  {
    updateSpeaker();

    if (isFinishedInit == 0) {
      continue;
    }
    
    //如果校準中不能進行其他操作
    if (mayCalibrateOnce() == HAL_BUSY) {
      continue;
    }

    HAL_StatusTypeDef status = mayUpdateMotionData();
    if (status == HAL_OK) {
      uint8_t motionPackage[7];
      generateMotionPackage(motionPackage);
      HAL_UART_Transmit(&huart1, motionPackage, sizeof(motionPackage), 200);
    }

    if (scannerState == SCANNER_DONE) {
      lastSuccessScanTime = HAL_GetTick();
      Vec2i16 viewerPosition = {0, 0};
      //Vec2i16 viewerPosition = {carPosition.x * 100, carPosition.y * 100};
      float heading = 360.0f - getCurrentHeadingByCache() - currentScanerAngle;
      if (heading >= 360.0f) heading -= 360.0f;
      else if (heading < 0.0f) heading += 360.0f;
      scannerReading.heading = heading;

      Vec2i16 obstaclePosition = getObstaclePosition(viewerPosition);
      setObstaclePixel(viewerPosition, obstaclePosition, settings_values[IDX_SCAN_DISTANCE]);

      //printCarFacingAndSensorFacing();

      swingScanner(settings_values[IDX_SCAN_CYCLE], settings_values[IDX_SCAN_RANGE], settings_values[IDX_SCAN_OFFSET]);
      HAL_Delay(50);
      scannerState = SCANNER_IDLE;
    }

    if (settings_values[IDX_ENABLE_SCAN] == 1) {
      if (SCANNER_IDLE == scannerState) startScan();
      if (HAL_GetTick() - lastSuccessScanTime > 1000) {
    	lastSuccessScanTime = HAL_GetTick();
        scannerState = SCANNER_IDLE;
        startScan();
      }
    } else {
      rotateScanner(settings_values[IDX_SCAN_OFFSET]);
    }

    uint8_t rxData;
    if (HAL_UART_Receive(&huart1, &rxData, 1, 20) == HAL_OK) {
      process_command(rxData);
    }

    updateMovementControl(
      (float)settings_values[IDX_MAX_MOVEMENT_SPEED] / settings[IDX_MAX_MOVEMENT_SPEED].max, 
      (uint8_t)settings_values[IDX_MAX_STEERING_ANGLE],
      300
    );

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HCSR04_Trig_GPIO_Port, HCSR04_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Speed_Motor_Ctrl_1_Pin|Speed_Motor_Ctrl_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Reset_BTN_Pin GY87_EXTI_Pin */
  GPIO_InitStruct.Pin = Reset_BTN_Pin|GY87_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Calibration_BTN_Pin */
  GPIO_InitStruct.Pin = Calibration_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Calibration_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HCSR04_Trig_Pin */
  GPIO_InitStruct.Pin = HCSR04_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HCSR04_Trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HC05_State_Pin */
  GPIO_InitStruct.Pin = HC05_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HC05_State_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HWM883L_DRDY_Pin */
  GPIO_InitStruct.Pin = HWM883L_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HWM883L_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Speed_Motor_Ctrl_1_Pin Speed_Motor_Ctrl_2_Pin */
  GPIO_InitStruct.Pin = Speed_Motor_Ctrl_1_Pin|Speed_Motor_Ctrl_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
