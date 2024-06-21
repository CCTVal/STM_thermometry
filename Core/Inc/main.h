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
#define keypadColumn1_Pin GPIO_PIN_5
#define keypadColumn1_GPIO_Port GPIOC
#define keypadRow4_Pin GPIO_PIN_1
#define keypadRow4_GPIO_Port GPIOB
#define keypadRow4_EXTI_IRQn EXTI1_IRQn
#define keypadRow1_Pin GPIO_PIN_13
#define keypadRow1_GPIO_Port GPIOB
#define keypadRow1_EXTI_IRQn EXTI15_10_IRQn
#define keypadRow2_Pin GPIO_PIN_14
#define keypadRow2_GPIO_Port GPIOB
#define keypadRow2_EXTI_IRQn EXTI15_10_IRQn
#define keypadRow3_Pin GPIO_PIN_15
#define keypadRow3_GPIO_Port GPIOB
#define keypadRow3_EXTI_IRQn EXTI15_10_IRQn
#define keypadColumn2_Pin GPIO_PIN_6
#define keypadColumn2_GPIO_Port GPIOC
#define SPI_CS2_Pin GPIO_PIN_7
#define SPI_CS2_GPIO_Port GPIOC
#define keypadColumn3_Pin GPIO_PIN_8
#define keypadColumn3_GPIO_Port GPIOC
#define SPI_CS4_Pin GPIO_PIN_8
#define SPI_CS4_GPIO_Port GPIOA
#define SPI_CS3_Pin GPIO_PIN_9
#define SPI_CS3_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_4
#define SPI3_CS_GPIO_Port GPIOB
#define SPI_CS1_Pin GPIO_PIN_6
#define SPI_CS1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
