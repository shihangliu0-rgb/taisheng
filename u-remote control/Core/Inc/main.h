/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define SW_2_Pin GPIO_PIN_3
#define SW_2_GPIO_Port GPIOE
#define SW_1_Pin GPIO_PIN_4
#define SW_1_GPIO_Port GPIOE
#define KEY_3_Pin GPIO_PIN_0
#define KEY_3_GPIO_Port GPIOC
#define KEY_2_Pin GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOC
#define KEY_1_Pin GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOA
#define KEY_4_Pin GPIO_PIN_1
#define KEY_4_GPIO_Port GPIOA
#define KEY_5_Pin GPIO_PIN_2
#define KEY_5_GPIO_Port GPIOA
#define KEY_6_Pin GPIO_PIN_4
#define KEY_6_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define KEY_10_Pin GPIO_PIN_8
#define KEY_10_GPIO_Port GPIOD
#define KEY_11_Pin GPIO_PIN_9
#define KEY_11_GPIO_Port GPIOD
#define KEY_12_Pin GPIO_PIN_10
#define KEY_12_GPIO_Port GPIOD
#define KEY_9_Pin GPIO_PIN_11
#define KEY_9_GPIO_Port GPIOD
#define KEY_8_Pin GPIO_PIN_12
#define KEY_8_GPIO_Port GPIOD
#define KEY_7_Pin GPIO_PIN_13
#define KEY_7_GPIO_Port GPIOD
#define SW_6_Pin GPIO_PIN_5
#define SW_6_GPIO_Port GPIOD
#define SW_5_Pin GPIO_PIN_6
#define SW_5_GPIO_Port GPIOD
#define SW_4_Pin GPIO_PIN_0
#define SW_4_GPIO_Port GPIOE
#define SW_3_Pin GPIO_PIN_1
#define SW_3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
