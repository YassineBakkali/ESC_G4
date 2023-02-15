/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define BLDC_PHASE_1_Pin GPIO_PIN_0
#define BLDC_PHASE_1_GPIO_Port GPIOC
#define BLDC_PHASE_2_Pin GPIO_PIN_1
#define BLDC_PHASE_2_GPIO_Port GPIOC
#define BLDC_PHASE_3_Pin GPIO_PIN_2
#define BLDC_PHASE_3_GPIO_Port GPIOC
#define CAN_TERM_Pin GPIO_PIN_3
#define CAN_TERM_GPIO_Port GPIOA
#define TEMP_BLDC_Pin GPIO_PIN_4
#define TEMP_BLDC_GPIO_Port GPIOA
#define Vbat_senes_Pin GPIO_PIN_5
#define Vbat_senes_GPIO_Port GPIOA
#define HALL1_Pin GPIO_PIN_0
#define HALL1_GPIO_Port GPIOB
#define HALL2_Pin GPIO_PIN_1
#define HALL2_GPIO_Port GPIOB
#define HALL3_Pin GPIO_PIN_2
#define HALL3_GPIO_Port GPIOB
#define CAL_Pin GPIO_PIN_11
#define CAL_GPIO_Port GPIOB
#define GLC_Pin GPIO_PIN_14
#define GLC_GPIO_Port GPIOB
#define GHC_Pin GPIO_PIN_15
#define GHC_GPIO_Port GPIOB
#define GHB_Pin GPIO_PIN_7
#define GHB_GPIO_Port GPIOC
#define GLA_Pin GPIO_PIN_8
#define GLA_GPIO_Port GPIOC
#define GHA_Pin GPIO_PIN_9
#define GHA_GPIO_Port GPIOC
#define Enable_Pin GPIO_PIN_8
#define Enable_GPIO_Port GPIOA
#define PWR_LED_Pin GPIO_PIN_6
#define PWR_LED_GPIO_Port GPIOB
#define FAULT_LED_Pin GPIO_PIN_7
#define FAULT_LED_GPIO_Port GPIOB
#define MOTOR_FAULT_Pin GPIO_PIN_9
#define MOTOR_FAULT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
