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
#define hbridge_l1_Pin GPIO_PIN_5
#define hbridge_l1_GPIO_Port GPIOA
#define hbridge_en_Pin GPIO_PIN_6
#define hbridge_en_GPIO_Port GPIOA
#define hbridge_l2_Pin GPIO_PIN_7
#define hbridge_l2_GPIO_Port GPIOA
#define stp2_dir_Pin GPIO_PIN_6
#define stp2_dir_GPIO_Port GPIOC
#define lin_servo_pwm_Pin GPIO_PIN_7
#define lin_servo_pwm_GPIO_Port GPIOC
#define stp2_pul_Pin GPIO_PIN_8
#define stp2_pul_GPIO_Port GPIOC
#define rot_servo_pwm_Pin GPIO_PIN_9
#define rot_servo_pwm_GPIO_Port GPIOC
#define stp1_pul_Pin GPIO_PIN_8
#define stp1_pul_GPIO_Port GPIOB
#define stp1_dir_Pin GPIO_PIN_9
#define stp1_dir_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
