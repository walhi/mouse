/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BTN4_Pin GPIO_PIN_2
#define BTN4_GPIO_Port GPIOA
#define RETURN_TO_BOOTLOADER_Pin GPIO_PIN_3
#define RETURN_TO_BOOTLOADER_GPIO_Port GPIOA
#define RIGHT_Pin GPIO_PIN_4
#define RIGHT_GPIO_Port GPIOA
#define UP_Pin GPIO_PIN_5
#define UP_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_6
#define LEFT_GPIO_Port GPIOA
#define DOWN_Pin GPIO_PIN_7
#define DOWN_GPIO_Port GPIOA
#define BTN5_Pin GPIO_PIN_0
#define BTN5_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_1
#define BTN3_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_10
#define BTN2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_11
#define BTN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define L_KEY_Pin BTN1_Pin
#define L_KEY_GPIO_Port BTN1_GPIO_Port
#define R_KEY_Pin BTN2_Pin
#define R_KEY_GPIO_Port BTN2_GPIO_Port
#define M_KEY_Pin BTN3_Pin
#define M_KEY_GPIO_Port BTN3_GPIO_Port
#define SCROLL_UP_Pin BTN4_Pin
#define SCROLL_UP_GPIO_Port BTN4_GPIO_Port
#define SCROLL_DOWN_Pin BTN5_Pin
#define SCROLL_DOWN_GPIO_Port BTN5_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
