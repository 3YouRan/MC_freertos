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
#define ST1_Pin GPIO_PIN_14
#define ST1_GPIO_Port GPIOB
#define EN1_Pin GPIO_PIN_15
#define EN1_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_11
#define DC_GPIO_Port GPIOD
#define RES_Pin GPIO_PIN_12
#define RES_GPIO_Port GPIOD
#define SDA_Pin GPIO_PIN_13
#define SDA_GPIO_Port GPIOD
#define SCL_Pin GPIO_PIN_14
#define SCL_GPIO_Port GPIOD
#define ST2_Pin GPIO_PIN_6
#define ST2_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_7
#define DIR2_GPIO_Port GPIOC
#define EN2_Pin GPIO_PIN_8
#define EN2_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_9
#define DIR1_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOC
#define LASER_Pin GPIO_PIN_2
#define LASER_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
