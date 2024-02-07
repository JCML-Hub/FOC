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
#define RGB_R_Pin GPIO_PIN_13
#define RGB_R_GPIO_Port GPIOC
#define RGB_G_Pin GPIO_PIN_14
#define RGB_G_GPIO_Port GPIOC
#define RGB_B_Pin GPIO_PIN_15
#define RGB_B_GPIO_Port GPIOC
#define SOC_Pin GPIO_PIN_0
#define SOC_GPIO_Port GPIOA
#define SOB_Pin GPIO_PIN_1
#define SOB_GPIO_Port GPIOA
#define SOA_Pin GPIO_PIN_2
#define SOA_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_3
#define TEMP_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define BOOT0_Pin GPIO_PIN_10
#define BOOT0_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_11
#define BOOT1_GPIO_Port GPIOB
#define FAULT_Pin GPIO_PIN_14
#define FAULT_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_15
#define EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
