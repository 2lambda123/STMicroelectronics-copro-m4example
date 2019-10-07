/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32mp1xx_hal.h"

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
void Error_Handler(char* file, int line);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SUB_NRST_Pin GPIO_PIN_10
#define SUB_NRST_GPIO_Port GPIOD
#define uSD_LDS_EN_Pin GPIO_PIN_11
#define uSD_LDS_EN_GPIO_Port GPIOE
#define LED_O_Pin GPIO_PIN_8
#define LED_O_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOD
#define MFX_IRQOUT_Pin GPIO_PIN_8
#define MFX_IRQOUT_GPIO_Port GPIOI
#define PMIC_WAKEUP_Pin GPIO_PIN_13
#define PMIC_WAKEUP_GPIO_Port GPIOC
#define PA14_Pin GPIO_PIN_14
#define PA14_GPIO_Port GPIOA
#define CAN_STBY_Pin GPIO_PIN_3
#define CAN_STBY_GPIO_Port GPIOG
#define uSD_LDO_SEL_Pin GPIO_PIN_14
#define uSD_LDO_SEL_GPIO_Port GPIOF
#define DSI_PWM_Pin GPIO_PIN_13
#define DSI_PWM_GPIO_Port GPIOD
#define PA13_Pin GPIO_PIN_13
#define PA13_GPIO_Port GPIOA
#define ETH_MDINT_Pin GPIO_PIN_0
#define ETH_MDINT_GPIO_Port GPIOG
#define DSI_RESET_Pin GPIO_PIN_15
#define DSI_RESET_GPIO_Port GPIOF
#define uSD_DETECT_Pin GPIO_PIN_1
#define uSD_DETECT_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
