/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    CALIBRATION_IDLE,
    CALIBRATION_REQUESTED,
    CALIBRATION_STATIC,
    CALIBRATION_MOVING,
    CALIBRATION_DONE,
} CalibrationState;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Reset_BTN_Pin GPIO_PIN_13
#define Reset_BTN_GPIO_Port GPIOC
#define Reset_BTN_EXTI_IRQn EXTI15_10_IRQn
#define Calibration_BTN_Pin GPIO_PIN_0
#define Calibration_BTN_GPIO_Port GPIOA
#define Calibration_BTN_EXTI_IRQn EXTI0_IRQn
#define HCSR04_Echo_Pin GPIO_PIN_2
#define HCSR04_Echo_GPIO_Port GPIOA
#define HCSR04_Trig_Pin GPIO_PIN_3
#define HCSR04_Trig_GPIO_Port GPIOA
#define HC05_State_Pin GPIO_PIN_5
#define HC05_State_GPIO_Port GPIOA
#define HC05_State_EXTI_IRQn EXTI9_5_IRQn
#define Dir_Motor_Control_Pin GPIO_PIN_6
#define Dir_Motor_Control_GPIO_Port GPIOA
#define Sensor_Motor_Control_Pin GPIO_PIN_7
#define Sensor_Motor_Control_GPIO_Port GPIOA
#define GY87_EXTI_Pin GPIO_PIN_4
#define GY87_EXTI_GPIO_Port GPIOC
#define GY87_EXTI_EXTI_IRQn EXTI4_IRQn
#define Speed_Motor_Control_Pin GPIO_PIN_13
#define Speed_Motor_Control_GPIO_Port GPIOE
#define HWM883L_SCL_Pin GPIO_PIN_10
#define HWM883L_SCL_GPIO_Port GPIOB
#define HWM883L_SDA_Pin GPIO_PIN_11
#define HWM883L_SDA_GPIO_Port GPIOB
#define HWM883L_DRDY_Pin GPIO_PIN_12
#define HWM883L_DRDY_GPIO_Port GPIOB
#define HWM883L_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define Speaker_PWM_Pin GPIO_PIN_13
#define Speaker_PWM_GPIO_Port GPIOD
#define HC05_TX_Pin GPIO_PIN_9
#define HC05_TX_GPIO_Port GPIOA
#define HC05_RX_Pin GPIO_PIN_10
#define HC05_RX_GPIO_Port GPIOA
#define Speed_Motor_Ctrl_1_Pin GPIO_PIN_4
#define Speed_Motor_Ctrl_1_GPIO_Port GPIOD
#define Speed_Motor_Ctrl_2_Pin GPIO_PIN_5
#define Speed_Motor_Ctrl_2_GPIO_Port GPIOD
#define GY87_SCL_Pin GPIO_PIN_6
#define GY87_SCL_GPIO_Port GPIOB
#define GY87_SDA_Pin GPIO_PIN_7
#define GY87_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
