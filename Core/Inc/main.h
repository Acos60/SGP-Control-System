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
#define PWM1_Pin GPIO_PIN_5
#define PWM1_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_6
#define PWM2_GPIO_Port GPIOE
#define M1_IN1_Pin GPIO_PIN_0
#define M1_IN1_GPIO_Port GPIOF
#define M1_IN2_Pin GPIO_PIN_1
#define M1_IN2_GPIO_Port GPIOF
#define M2_IN1_Pin GPIO_PIN_2
#define M2_IN1_GPIO_Port GPIOF
#define M2_IN2_Pin GPIO_PIN_3
#define M2_IN2_GPIO_Port GPIOF
#define M3_IN1_Pin GPIO_PIN_4
#define M3_IN1_GPIO_Port GPIOF
#define M3_IN2_Pin GPIO_PIN_5
#define M3_IN2_GPIO_Port GPIOF
#define PWM3_Pin GPIO_PIN_6
#define PWM3_GPIO_Port GPIOF
#define PWM4_Pin GPIO_PIN_7
#define PWM4_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOF
#define M4_IN1_Pin GPIO_PIN_0
#define M4_IN1_GPIO_Port GPIOC
#define M4_IN2_Pin GPIO_PIN_1
#define M4_IN2_GPIO_Port GPIOC
#define Hall5_A_Pin GPIO_PIN_0
#define Hall5_A_GPIO_Port GPIOA
#define Hall5_B_Pin GPIO_PIN_1
#define Hall5_B_GPIO_Port GPIOA
#define Hall2_A_Pin GPIO_PIN_5
#define Hall2_A_GPIO_Port GPIOA
#define Hall3_A_Pin GPIO_PIN_6
#define Hall3_A_GPIO_Port GPIOA
#define Hall3_B_Pin GPIO_PIN_7
#define Hall3_B_GPIO_Port GPIOA
#define Hall1_A_Pin GPIO_PIN_9
#define Hall1_A_GPIO_Port GPIOE
#define Hall1_B_Pin GPIO_PIN_11
#define Hall1_B_GPIO_Port GPIOE
#define M6_IN1_Pin GPIO_PIN_12
#define M6_IN1_GPIO_Port GPIOB
#define M6_IN2_Pin GPIO_PIN_13
#define M6_IN2_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_14
#define PWM5_GPIO_Port GPIOB
#define PWM6_Pin GPIO_PIN_15
#define PWM6_GPIO_Port GPIOB
#define M5_IN1_Pin GPIO_PIN_8
#define M5_IN1_GPIO_Port GPIOD
#define M5_IN2_Pin GPIO_PIN_9
#define M5_IN2_GPIO_Port GPIOD
#define Hall4_A_Pin GPIO_PIN_12
#define Hall4_A_GPIO_Port GPIOD
#define Hall4_B_Pin GPIO_PIN_13
#define Hall4_B_GPIO_Port GPIOD
#define Hall6_A_Pin GPIO_PIN_6
#define Hall6_A_GPIO_Port GPIOC
#define Hall6_B_Pin GPIO_PIN_7
#define Hall6_B_GPIO_Port GPIOC
#define Hall2_B_Pin GPIO_PIN_3
#define Hall2_B_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_8
#define KEY1_GPIO_Port GPIOB
#define KEY0_Pin GPIO_PIN_9
#define KEY0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
