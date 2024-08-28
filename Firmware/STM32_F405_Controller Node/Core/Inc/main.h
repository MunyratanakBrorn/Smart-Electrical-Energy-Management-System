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
#define CON_CoilRL_Pin GPIO_PIN_13
#define CON_CoilRL_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define TRI2_Pin GPIO_PIN_10
#define TRI2_GPIO_Port GPIOB
#define TRI1_Pin GPIO_PIN_11
#define TRI1_GPIO_Port GPIOB
#define LED_A_Pin GPIO_PIN_12
#define LED_A_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define LED_C_Pin GPIO_PIN_14
#define LED_C_GPIO_Port GPIOB
#define LED_D_Pin GPIO_PIN_15
#define LED_D_GPIO_Port GPIOB
#define RL4_Pin GPIO_PIN_6
#define RL4_GPIO_Port GPIOC
#define RL3_Pin GPIO_PIN_7
#define RL3_GPIO_Port GPIOC
#define RL2_Pin GPIO_PIN_8
#define RL2_GPIO_Port GPIOC
#define RL1_Pin GPIO_PIN_9
#define RL1_GPIO_Port GPIOC
#define RL5_Pin GPIO_PIN_8
#define RL5_GPIO_Port GPIOA
#define Zero_Cross2_Pin GPIO_PIN_8
#define Zero_Cross2_GPIO_Port GPIOB
#define Zero_Cross1_Pin GPIO_PIN_9
#define Zero_Cross1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
