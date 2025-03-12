/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi.h"
#include "pmw3901.h"
#include "string.h"
#include "usart.h"
#include "gpio.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "string.h"
#include "tim.h"
#include "motor.h"
#include "tasks.h"
#include "control.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "queueHandle.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOF_GPIO_Pin GPIO_PIN_1
#define TOF_GPIO_GPIO_Port GPIOB
#define TOF_XSHUT_Pin GPIO_PIN_2
#define TOF_XSHUT_GPIO_Port GPIOB
#define TOF_SCL_Pin GPIO_PIN_10
#define TOF_SCL_GPIO_Port GPIOB
#define TOF_SDA_Pin GPIO_PIN_11
#define TOF_SDA_GPIO_Port GPIOB
#define AIN21_PIN_Pin GPIO_PIN_8
#define AIN21_PIN_GPIO_Port GPIOC
#define STBY2_PIN_Pin GPIO_PIN_9
#define STBY2_PIN_GPIO_Port GPIOC
#define BIN22_PIN_Pin GPIO_PIN_8
#define BIN22_PIN_GPIO_Port GPIOA
#define BIN21_PIN_Pin GPIO_PIN_11
#define BIN21_PIN_GPIO_Port GPIOA
#define AIN22_PIN_Pin GPIO_PIN_12
#define AIN22_PIN_GPIO_Port GPIOA
#define BIN2_PIN_Pin GPIO_PIN_12
#define BIN2_PIN_GPIO_Port GPIOC
#define BIN1_PIN_Pin GPIO_PIN_2
#define BIN1_PIN_GPIO_Port GPIOD
#define AIN1_PIN_Pin GPIO_PIN_3
#define AIN1_PIN_GPIO_Port GPIOB
#define AIN2_PIN_Pin GPIO_PIN_4
#define AIN2_PIN_GPIO_Port GPIOB
#define STBY_PIN_Pin GPIO_PIN_5
#define STBY_PIN_GPIO_Port GPIOB
#define PIN_CS_Pin GPIO_PIN_7
#define PIN_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
