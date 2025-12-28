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
#include "stm32f7xx_hal.h"

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
#define PCM_PUI_Pin GPIO_PIN_13
#define PCM_PUI_GPIO_Port GPIOC
#define SP_Pin GPIO_PIN_0
#define SP_GPIO_Port GPIOF
#define HP_Pin GPIO_PIN_1
#define HP_GPIO_Port GPIOF
#define MIC_A_R_Pin GPIO_PIN_2
#define MIC_A_R_GPIO_Port GPIOF
#define GAIN_Pin GPIO_PIN_3
#define GAIN_GPIO_Port GPIOF
#define PTT_Pin GPIO_PIN_4
#define PTT_GPIO_Port GPIOF
#define PTT_EXTI_IRQn EXTI4_IRQn
#define MIC_Pin GPIO_PIN_5
#define MIC_GPIO_Port GPIOF
#define DEV_ADDR_Pin GPIO_PIN_6
#define DEV_ADDR_GPIO_Port GPIOF
#define SD_CD_Pin GPIO_PIN_2
#define SD_CD_GPIO_Port GPIOC
#define SD_CD_EXTI_IRQn EXTI2_IRQn
#define W5500_IRQ_Pin GPIO_PIN_0
#define W5500_IRQ_GPIO_Port GPIOA
#define W5500_IRQ_EXTI_IRQn EXTI0_IRQn
#define W5500_NSS_Pin GPIO_PIN_4
#define W5500_NSS_GPIO_Port GPIOA
#define W5500_RESET_Pin GPIO_PIN_5
#define W5500_RESET_GPIO_Port GPIOC
#define LOW_BATTERY_Pin GPIO_PIN_1
#define LOW_BATTERY_GPIO_Port GPIOG
#define LOW_BATTERY_EXTI_IRQn EXTI1_IRQn
#define CALL_BTN_Pin GPIO_PIN_8
#define CALL_BTN_GPIO_Port GPIOE
#define CALL_BTN_EXTI_IRQn EXTI9_5_IRQn
#define ALARM_BTN_Pin GPIO_PIN_9
#define ALARM_BTN_GPIO_Port GPIOE
#define ALARM_BTN_EXTI_IRQn EXTI9_5_IRQn
#define RESET_ALARM_BTN_Pin GPIO_PIN_10
#define RESET_ALARM_BTN_GPIO_Port GPIOE
#define RESET_ALARM_BTN_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR_1_Pin GPIO_PIN_11
#define SENSOR_1_GPIO_Port GPIOE
#define SENSOR_1_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR_2_Pin GPIO_PIN_12
#define SENSOR_2_GPIO_Port GPIOE
#define SENSOR_2_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR_3_Pin GPIO_PIN_13
#define SENSOR_3_GPIO_Port GPIOE
#define SENSOR_3_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR_4_Pin GPIO_PIN_14
#define SENSOR_4_GPIO_Port GPIOE
#define SENSOR_4_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR_5_Pin GPIO_PIN_15
#define SENSOR_5_GPIO_Port GPIOE
#define SENSOR_5_EXTI_IRQn EXTI15_10_IRQn
#define LED_GROUP_SENSOR_Pin GPIO_PIN_10
#define LED_GROUP_SENSOR_GPIO_Port GPIOB
#define LED_SENSOR_1_Pin GPIO_PIN_11
#define LED_SENSOR_1_GPIO_Port GPIOB
#define LED_SENSOR_2_Pin GPIO_PIN_12
#define LED_SENSOR_2_GPIO_Port GPIOB
#define LED_SENSOR_3_Pin GPIO_PIN_13
#define LED_SENSOR_3_GPIO_Port GPIOB
#define LED_SENSOR_4_Pin GPIO_PIN_14
#define LED_SENSOR_4_GPIO_Port GPIOB
#define LED_SENSOR_5_Pin GPIO_PIN_15
#define LED_SENSOR_5_GPIO_Port GPIOB
#define LED_ALARM_Pin GPIO_PIN_8
#define LED_ALARM_GPIO_Port GPIOD
#define LED_SD_CARD_Pin GPIO_PIN_9
#define LED_SD_CARD_GPIO_Port GPIOD
#define LED_CALL_Pin GPIO_PIN_10
#define LED_CALL_GPIO_Port GPIOD
#define BCLKR_CTRL_Pin GPIO_PIN_1
#define BCLKR_CTRL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
