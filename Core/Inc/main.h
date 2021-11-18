/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUCK_EN_Pin GPIO_PIN_13
#define BUCK_EN_GPIO_Port GPIOC
#define RFID_EN_Pin GPIO_PIN_14
#define RFID_EN_GPIO_Port GPIOC
#define LOCK_CONTROL_Pin GPIO_PIN_15
#define LOCK_CONTROL_GPIO_Port GPIOC
#define XIN_Pin GPIO_PIN_0
#define XIN_GPIO_Port GPIOH
#define RFID_IRQ_Pin GPIO_PIN_4
#define RFID_IRQ_GPIO_Port GPIOC
#define RFID_IRQ_EXTI_IRQn EXTI4_IRQn
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define UNUSED_IO_8_Pin GPIO_PIN_13
#define UNUSED_IO_8_GPIO_Port GPIOB
#define UNUSED_IO_7_Pin GPIO_PIN_14
#define UNUSED_IO_7_GPIO_Port GPIOB
#define UNUSED_IO_6_Pin GPIO_PIN_15
#define UNUSED_IO_6_GPIO_Port GPIOB
#define UNUSED_IO_5_Pin GPIO_PIN_6
#define UNUSED_IO_5_GPIO_Port GPIOC
#define UNISED_IO_4_Pin GPIO_PIN_7
#define UNISED_IO_4_GPIO_Port GPIOC
#define UNUSED_IO_3_Pin GPIO_PIN_8
#define UNUSED_IO_3_GPIO_Port GPIOC
#define UNUSED_IO_2_Pin GPIO_PIN_9
#define UNUSED_IO_2_GPIO_Port GPIOC
#define PWM_BUZZER_Pin GPIO_PIN_8
#define PWM_BUZZER_GPIO_Port GPIOA
#define UNUSED_IO_1_Pin GPIO_PIN_9
#define UNUSED_IO_1_GPIO_Port GPIOA
#define BOOT0_Pin GPIO_PIN_3
#define BOOT0_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
