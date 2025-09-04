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
#include "stm32f0xx_hal.h"

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
#define LCD_MAX_TIMEOUT 12

#define GSM_RI_Pin GPIO_PIN_0
#define GSM_RI_GPIO_Port GPIOA
#define GSM_DTR_Pin GPIO_PIN_1
#define GSM_DTR_GPIO_Port GPIOA
#define PUMP_R_Pin GPIO_PIN_4
#define PUMP_R_GPIO_Port GPIOA
#define SW_5_Pin GPIO_PIN_5
#define SW_5_GPIO_Port GPIOA
#define SW_5_EXTI_IRQn EXTI4_15_IRQn
#define SW_4_Pin GPIO_PIN_6
#define SW_4_GPIO_Port GPIOA
#define SW_4_EXTI_IRQn EXTI4_15_IRQn
#define SW_3_Pin GPIO_PIN_7
#define SW_3_GPIO_Port GPIOA
#define SW_3_EXTI_IRQn EXTI4_15_IRQn
#define SW_2_Pin GPIO_PIN_0
#define SW_2_GPIO_Port GPIOB
#define SW_2_EXTI_IRQn EXTI0_1_IRQn
#define SW_1_Pin GPIO_PIN_1
#define SW_1_GPIO_Port GPIOB
#define SW_1_EXTI_IRQn EXTI0_1_IRQn
#define EXP_OUT_Pin GPIO_PIN_2
#define EXP_OUT_GPIO_Port GPIOB
#define RELAY_PWR_Pin GPIO_PIN_8
#define RELAY_PWR_GPIO_Port GPIOA
#define MAINS_SN_Pin GPIO_PIN_9
#define MAINS_SN_GPIO_Port GPIOA
#define FLOW_1_Pin GPIO_PIN_10
#define FLOW_1_GPIO_Port GPIOA
#define FLOW_1_EXTI_IRQn EXTI4_15_IRQn
#define LOW_FLOW_Pin GPIO_PIN_11
#define LOW_FLOW_GPIO_Port GPIOA
#define RAIN_SENSOR_Pin GPIO_PIN_12
#define RAIN_SENSOR_GPIO_Port GPIOA
#define LCD_LED_Pin GPIO_PIN_15  // ++$
#define LCD_LED_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_3
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_4
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_5
#define LCD_D4_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_8
#define LCD_EN_GPIO_Port GPIOB
#define PWR_MODEM_Pin  GPIO_PIN_13 // ++$
#define PWR_MODEM_GPIO_Port GPIOB
#define MODEM_REG_Pin  GPIO_PIN_14
#define MODEM_REG_GPIO_Port GPIOB



//#define LCD_LED_Pin GPIO_PIN_8
//#define LCD_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
