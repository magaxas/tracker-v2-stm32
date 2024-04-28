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
#include "stm32l0xx_hal.h"

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
#define BLUE_LED_Pin GPIO_PIN_14
#define BLUE_LED_GPIO_Port GPIOC
#define RED_LED_Pin GPIO_PIN_15
#define RED_LED_GPIO_Port GPIOC
#define ADC_IN_Pin GPIO_PIN_0
#define ADC_IN_GPIO_Port GPIOA
#define PUSH_BTN_Pin GPIO_PIN_1
#define PUSH_BTN_GPIO_Port GPIOA
#define PUSH_BTN_EXTI_IRQn EXTI0_1_IRQn
#define PWRKEY_Pin GPIO_PIN_4
#define PWRKEY_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_0
#define SD_CS_GPIO_Port GPIOB
#define MAIN_DTR_Pin GPIO_PIN_1
#define MAIN_DTR_GPIO_Port GPIOB
#define ACCEL_INT1_Pin GPIO_PIN_8
#define ACCEL_INT1_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_11
#define GREEN_LED_GPIO_Port GPIOA
#define GNNS_LNA_EN_Pin GPIO_PIN_12
#define GNNS_LNA_EN_GPIO_Port GPIOA
#define FPWM_EN_Pin GPIO_PIN_4
#define FPWM_EN_GPIO_Port GPIOB
#define BUCK_BOOST_EN_Pin GPIO_PIN_5
#define BUCK_BOOST_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BUF_MAX_LEN 1000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
