/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define H1_NSLEEP_Pin GPIO_PIN_13
#define H1_NSLEEP_GPIO_Port GPIOC
#define H1_PHASE_Pin GPIO_PIN_14
#define H1_PHASE_GPIO_Port GPIOC
#define H1_MODE_Pin GPIO_PIN_15
#define H1_MODE_GPIO_Port GPIOC
#define H2_FAULT_Pin GPIO_PIN_0
#define H2_FAULT_GPIO_Port GPIOC
#define H1_FAULT_Pin GPIO_PIN_1
#define H1_FAULT_GPIO_Port GPIOC
#define H2_VPROPI_Pin GPIO_PIN_2
#define H2_VPROPI_GPIO_Port GPIOC
#define H1_VPROPI_Pin GPIO_PIN_3
#define H1_VPROPI_GPIO_Port GPIOC
#define H4_VPROPI_Pin GPIO_PIN_4
#define H4_VPROPI_GPIO_Port GPIOA
#define H3_VPROPI_Pin GPIO_PIN_4
#define H3_VPROPI_GPIO_Port GPIOC
#define H3_FAULT_Pin GPIO_PIN_5
#define H3_FAULT_GPIO_Port GPIOC
#define BATT_VOLTAGE_Pin GPIO_PIN_0
#define BATT_VOLTAGE_GPIO_Port GPIOB
#define IMU_INT_1_Pin GPIO_PIN_1
#define IMU_INT_1_GPIO_Port GPIOB
#define H3_NSLEEP_Pin GPIO_PIN_12
#define H3_NSLEEP_GPIO_Port GPIOB
#define H3_PHASE_Pin GPIO_PIN_13
#define H3_PHASE_GPIO_Port GPIOB
#define H4_FAULT_Pin GPIO_PIN_14
#define H4_FAULT_GPIO_Port GPIOB
#define IMU_INT_2_Pin GPIO_PIN_6
#define IMU_INT_2_GPIO_Port GPIOC
#define H3_MODE_Pin GPIO_PIN_8
#define H3_MODE_GPIO_Port GPIOC
#define H4_MODE_Pin GPIO_PIN_9
#define H4_MODE_GPIO_Port GPIOC
#define H4_NSLEEP_Pin GPIO_PIN_12
#define H4_NSLEEP_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define H4_PHASE_Pin GPIO_PIN_11
#define H4_PHASE_GPIO_Port GPIOC
#define H2_MODE_Pin GPIO_PIN_12
#define H2_MODE_GPIO_Port GPIOC
#define H2_NSLEEP_Pin GPIO_PIN_4
#define H2_NSLEEP_GPIO_Port GPIOB
#define H2_PHASE_Pin GPIO_PIN_5
#define H2_PHASE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
