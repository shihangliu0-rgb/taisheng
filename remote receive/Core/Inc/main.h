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
#include "stm32f1xx_hal.h"

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
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOC
#define UART2_TX_Pin GPIO_PIN_2
#define UART2_TX_GPIO_Port GPIOA
#define UART2_RX_Pin GPIO_PIN_3
#define UART2_RX_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define LORA_M0_Pin GPIO_PIN_0
#define LORA_M0_GPIO_Port GPIOB
#define LORA_M1_Pin GPIO_PIN_1
#define LORA_M1_GPIO_Port GPIOB
#define LORA_TX_Pin GPIO_PIN_10
#define LORA_TX_GPIO_Port GPIOB
#define LORA_RX_Pin GPIO_PIN_11
#define LORA_RX_GPIO_Port GPIOB
#define LORA_AUX_Pin GPIO_PIN_12
#define LORA_AUX_GPIO_Port GPIOB
#define U4_11_PB13_Pin GPIO_PIN_13
#define U4_11_PB13_GPIO_Port GPIOB
#define U4_12_PB14_Pin GPIO_PIN_14
#define U4_12_PB14_GPIO_Port GPIOB
#define U4_9_PB15_Pin GPIO_PIN_15
#define U4_9_PB15_GPIO_Port GPIOB
#define U4_10_PA8_Pin GPIO_PIN_8
#define U4_10_PA8_GPIO_Port GPIOA
#define U4_7_PA9_Pin GPIO_PIN_9
#define U4_7_PA9_GPIO_Port GPIOA
#define U4_8_PA10_Pin GPIO_PIN_10
#define U4_8_PA10_GPIO_Port GPIOA
#define U4_5_PA11_Pin GPIO_PIN_11
#define U4_5_PA11_GPIO_Port GPIOA
#define U4_6_PA12_Pin GPIO_PIN_12
#define U4_6_PA12_GPIO_Port GPIOA
#define U1_4_PA15_Pin GPIO_PIN_15
#define U1_4_PA15_GPIO_Port GPIOA
#define U1_3_PB3_Pin GPIO_PIN_3
#define U1_3_PB3_GPIO_Port GPIOB
#define U1_6_PB4_Pin GPIO_PIN_4
#define U1_6_PB4_GPIO_Port GPIOB
#define U1_5_PB5_Pin GPIO_PIN_5
#define U1_5_PB5_GPIO_Port GPIOB
#define U1_8_PB6_Pin GPIO_PIN_6
#define U1_8_PB6_GPIO_Port GPIOB
#define U1_7_PB7_Pin GPIO_PIN_7
#define U1_7_PB7_GPIO_Port GPIOB
#define U1_10_PB8_Pin GPIO_PIN_8
#define U1_10_PB8_GPIO_Port GPIOB
#define U1_9_PB9_Pin GPIO_PIN_9
#define U1_9_PB9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
