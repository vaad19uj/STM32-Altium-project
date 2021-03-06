/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "14443.h"
#include "parallel.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
int interruptFlag = 0;
unsigned char command[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 =======================================================================================================================
    Function writes only one register or a multiple number ;
    of registers with specified addresses ;
 =======================================================================================================================
 */
void WriteSingle(unsigned char *pbuf, unsigned char length)
{
	*pbuf = (0x1f &*pbuf);	/* register address */
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&*pbuf, length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, SET);
}		/* WriteSingle */

/*
 =======================================================================================================================
    Function writes a specified number of registers from ;
    a specified address upwards ;
 =======================================================================================================================
 */
void WriteCont(unsigned char *pbuf, unsigned char length)
{
    *pbuf = (0x20 | *pbuf); /* address, write, continous */
    *pbuf = (0x3f &*pbuf);	/* register address */
  	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, RESET);
  	HAL_SPI_Transmit(&hspi1, (uint8_t *)&*pbuf, length, HAL_MAX_DELAY);
  	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, SET);

}	/* WriteCont */

void RAWwrite(unsigned char *pbuf, unsigned char length)
{
  	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, RESET);
  	HAL_SPI_Transmit(&hspi1, (uint8_t *)&*pbuf, length, HAL_MAX_DELAY);
  	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, SET);

}	/* RAWwrite */

/*
 =======================================================================================================================
    Function DirectCommand transmits a command to the reader chip
 =======================================================================================================================
 */
void DirectCommand(unsigned char *pbuf)
{
    *pbuf = (0x80 | *pbuf); /* command */
    *pbuf = (0x9f &*pbuf);	/* command code */

    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&*pbuf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, SET);

}	/* DirectCommand */

void ReadCont(unsigned char *pbuf, unsigned char length)
{

}	/* ReadCont */

//RFID-FUNKTIONER vi beh??ver:
/*	host.c:
 * 		kputchar()
 * 		Put_byte()
 * 		Nibble2Ascii()
 *
 *	14443.c:
 *		AntiCollisionSequenceA()
 *		AnticollisionLoopA()
 *		SelectCommand()
 *
 * 	anticollision.c:
 * 		RequestCommand()
 *
 * 	parallell.c:
 * 		DirectCommand()
 * 		ReadCont()
 * 		WriteCont()
 * 		RAWwrite()
 * 		WriteSingle()
 *
 * CounterSet(); funktionen borde g?? att ers??tta med HAL_Delay() ist??llet
 *
 */

void led(){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
}

void checkIfCardIsPresent(){
	//set registers for protocol and execute anti collision sequence to find tags

	// ISO14443A
	command[0] = ChipStateControl;
	command[1] = 0x21;
	command[2] = ISOControl;	// set register 0x01 for ISO14443A operation
	command[3] = 0x08;
	WriteSingle(command, 4);
	HAL_Delay(5);

	AnticollisionSequenceA(0x00);	// do a complete anti collision sequence as described

	// in ISO14443-3 standard for type A
	command[0] = ChipStateControl;	///* turn off RF driver
	command[1] = 0x01;
	WriteSingle(command, 2);
	HAL_Delay(1);

	command[0] = IRQStatus;
	command[1] = IRQMask;
	ReadCont(command, 2);
}

int checkIfCardIdIsValid(){
	//do something with hspi1
	return 1;
}

void openAndCloseLock(){
	HAL_GPIO_WritePin(LOCK_CONTROL_GPIO_Port, LOCK_CONTROL_Pin, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LOCK_CONTROL_GPIO_Port, LOCK_CONTROL_Pin, RESET);
}

void buzzer(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	uint16_t new_pwm_val = 4999;

	// update PWM
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_pwm_val);
	HAL_Delay(3000);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

/*void RFIDLockLoop(){
	if(checkIfCardIsPresent()){
		if(checkIfCardIdIsValid()){
			openAndCloseLock();
		}else{
			led();
		}
	}
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		interruptFlag = 1;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  // Buck converter is turned on
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, RESET);

  //SPI SS-pin should default high
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, SET);

  // start timer 1 for interrupt
  HAL_TIM_Base_Start_IT(&htim1);

  // RFID chip enable input
  HAL_GPIO_WritePin(RFID_EN_GPIO_Port, RFID_EN_Pin, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // enter sleep mode, wake up from interrupt
	 HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	 if(interruptFlag){
		// RFIDLockLoop();
		 interruptFlag = 0;
	 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUCK_EN_Pin|RFID_EN_Pin|LOCK_CONTROL_Pin|UNUSED_IO_5_Pin
                          |UNISED_IO_4_Pin|UNUSED_IO_3_Pin|UNUSED_IO_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin|UNUSED_IO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|UNUSED_IO_8_Pin|UNUSED_IO_7_Pin|UNUSED_IO_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUCK_EN_Pin RFID_EN_Pin LOCK_CONTROL_Pin UNUSED_IO_5_Pin
                           UNISED_IO_4_Pin UNUSED_IO_3_Pin UNUSED_IO_2_Pin */
  GPIO_InitStruct.Pin = BUCK_EN_Pin|RFID_EN_Pin|LOCK_CONTROL_Pin|UNUSED_IO_5_Pin
                          |UNISED_IO_4_Pin|UNUSED_IO_3_Pin|UNUSED_IO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_NSS_Pin UNUSED_IO_1_Pin */
  GPIO_InitStruct.Pin = SPI_NSS_Pin|UNUSED_IO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID_IRQ_Pin */
  GPIO_InitStruct.Pin = RFID_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RFID_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin UNUSED_IO_8_Pin UNUSED_IO_7_Pin UNUSED_IO_6_Pin */
  GPIO_InitStruct.Pin = LED_Pin|UNUSED_IO_8_Pin|UNUSED_IO_7_Pin|UNUSED_IO_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_Pin */
  GPIO_InitStruct.Pin = BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
