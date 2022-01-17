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

void WriteSingle(unsigned char *pbuf, unsigned char length);
void WriteCont(unsigned char *pbuf, unsigned char length);
void ReadCont(unsigned char *pbuf, unsigned char length);
void DirectCommand(unsigned char *pbuf);
void RAWwrite(unsigned char *pbuf, unsigned char length);

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
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
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

//=======TRF definitions=========================
//Reader addresses
#define ChipStateControl	0x00
#define ISOControl			0x01
#define ISO14443Boptions	0x02
#define ISO14443Aoptions	0x03
#define TXtimerEPChigh		0x04
#define TXtimerEPClow		0x05
#define TXPulseLenghtControl	0x06
#define RXNoResponseWaitTime	0x07
#define RXWaitTime			0x08
#define ModulatorControl	0x09
#define RXSpecialSettings	0x0A
#define RegulatorControl	0x0B
#define IRQStatus			0x0C
#define IRQMask				0x0D
#define	CollisionPosition	0x0E
#define RSSILevels			0x0F
#define RAMStartAddress		0x10	//RAM is 7 bytes long (0x10 - 0x16)
#define NFCID				0x17
#define NFCTargetLevel		0x18
#define NFCTargetProtocol	0x19
#define TestSetting1		0x1A
#define TestSetting2		0x1B
#define FIFOStatus			0x1C
#define TXLenghtByte1		0x1D
#define TXLenghtByte2		0x1E
#define FIFO				0x1F

//Reader commands-------------------------------------------
#define Idle				0x00
#define SoftInit			0x03
#define InitialRFCollision	0x04
#define ResponseRFCollisionN	0x05
#define ResponseRFCollision0	0x06
#define	Reset				0x0F
#define TransmitNoCRC		0x10
#define TransmitCRC			0x11
#define DelayTransmitNoCRC	0x12
#define DelayTransmitCRC	0x13
#define TransmitNextSlot	0x14
#define CloseSlotSequence	0x15
#define StopDecoders		0x16
#define RunDecoders			0x17
#define ChectInternalRF		0x18
#define CheckExternalRF		0x19
#define AdjustGain			0x1A
//==========================================================

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
