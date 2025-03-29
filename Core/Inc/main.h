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
#define PWMB1_Pin GPIO_PIN_5
#define PWMB1_GPIO_Port GPIOE
#define PWMB2_Pin GPIO_PIN_6
#define PWMB2_GPIO_Port GPIOE
#define DAT_Pin GPIO_PIN_0
#define DAT_GPIO_Port GPIOC
#define CMD_Pin GPIO_PIN_1
#define CMD_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOC
#define CLK_Pin GPIO_PIN_3
#define CLK_GPIO_Port GPIOC
#define E4A_Pin GPIO_PIN_0
#define E4A_GPIO_Port GPIOA
#define E4B_Pin GPIO_PIN_1
#define E4B_GPIO_Port GPIOA
#define PWMC1_Pin GPIO_PIN_9
#define PWMC1_GPIO_Port GPIOE
#define PWMC2_Pin GPIO_PIN_11
#define PWMC2_GPIO_Port GPIOE
#define PWMD1_Pin GPIO_PIN_13
#define PWMD1_GPIO_Port GPIOE
#define PWMD2_Pin GPIO_PIN_14
#define PWMD2_GPIO_Port GPIOE
#define R1_Pin GPIO_PIN_14
#define R1_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_15
#define L1_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_11
#define DC_GPIO_Port GPIOD
#define RES_Pin GPIO_PIN_12
#define RES_GPIO_Port GPIOD
#define SDA_Pin GPIO_PIN_13
#define SDA_GPIO_Port GPIOD
#define SCL_Pin GPIO_PIN_14
#define SCL_GPIO_Port GPIOD
#define LED_L_Pin GPIO_PIN_15
#define LED_L_GPIO_Port GPIOD
#define SERVO2_Pin GPIO_PIN_8
#define SERVO2_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_9
#define SERVO1_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_15
#define E1A_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_12
#define L2_GPIO_Port GPIOC
#define R2_Pin GPIO_PIN_2
#define R2_GPIO_Port GPIOD
#define Motor_Enable_Pin GPIO_PIN_3
#define Motor_Enable_GPIO_Port GPIOD
#define E1B_Pin GPIO_PIN_3
#define E1B_GPIO_Port GPIOB
#define E2A_Pin GPIO_PIN_4
#define E2A_GPIO_Port GPIOB
#define E2B_Pin GPIO_PIN_5
#define E2B_GPIO_Port GPIOB
#define E3A_Pin GPIO_PIN_6
#define E3A_GPIO_Port GPIOB
#define E3B_Pin GPIO_PIN_7
#define E3B_GPIO_Port GPIOB
#define PWMA1_Pin GPIO_PIN_8
#define PWMA1_GPIO_Port GPIOB
#define PWMA2_Pin GPIO_PIN_9
#define PWMA2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
