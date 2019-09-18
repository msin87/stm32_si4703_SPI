/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t read[2] ={0x70,0x00};
uint8_t Si4703_REGs[4]={0};
uint16_t result[2];
void Si4703_SPI_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = Si4703_SDIO_Pin;					//config GPIOB for SDIO operation
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Si4703_SDIO_GPIO_Port, &GPIO_InitStruct);	//init GPIOB
	GPIO_InitStruct.Pin = Si4703_nReset_Pin|Si4703_nSen_Pin|Si4703_GPIO1_Pin;					//config GPIOС for nReset, nSen operation
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Si4703_nReset_GPIO_Port, &GPIO_InitStruct);

	Si4703_SDIO_GPIO_Port->BSRR|=Si4703_SDIO_Pin<<16;		//set SDIO to 0
	Si4703_nSen_GPIO_Port->BSRR|=Si4703_nSen_Pin<<16;		//set nSen to 0
	Si4703_nReset_GPIO_Port->BSRR|=Si4703_nReset_Pin<<16;	        //set nReset to 0 (start Si4703)
	HAL_Delay(5);
	Si4703_nReset_GPIO_Port->BSRR|=Si4703_nReset_Pin;		//set nReset to 1 (start Si4703)
	Si4703_nSen_GPIO_Port->BSRR|=Si4703_nSen_Pin;
	HAL_Delay(1);



}
void reverseBytes(uint8_t* array,uint8_t size)
{
  for (uint8_t i=0; i<size; i++)
  {
    array[i]=(__RBIT(array[i]))>>23;
  }
}
uint32_t getUs(void)
{
  uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
  register uint32_t ms, cycle_cnt;
  do
  {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  }
  while (ms != HAL_GetTick());
  return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}
void delayUs(uint16_t micros) {
  uint32_t start = getUs();
  while (getUs()-start < (uint32_t) micros)
  {
    asm("nop");
  }
}
uint16_t Si4703_SPI_Receive(uint8_t addr){
  uint8_t controlWord=0x70|(addr>>1);
  uint8_t bit9=addr&1;
  Si4703_nSen_GPIO_Port->BSRR|=Si4703_nSen_Pin<<16; 		//nSen to 0. Start
  HAL_SPI_Transmit(&hspi1, &controlWord, 1, 1000);		//transmit 8 bits
  Si4703_SDIO_GPIO_Port->BSRR|=Si4703_SDIO_Pin<<(16*(1-bit9)); 	//shift = 16 (reset) if bit9 (a0) = 0
  Si4703_SCLK_GPIO_Port->BSRR|=Si4703_SCLK_Pin; 		//SCLK to 1
  Si4703_SDIO_GPIO_Port->MODER^=(3<<(7*2)); 			//SDIO set as Output
  Si4703_SCLK_GPIO_Port->MODER^=(3<<(5*2)); 			//SCLK set as Output

  Si4703_SCLK_GPIO_Port->BSRR|=Si4703_SCLK_Pin<<16;     	//SCLK to 0. Prepare
  delayUs(1);
  Si4703_SCLK_GPIO_Port->BSRR|=Si4703_SCLK_Pin; 		//SCLK to 1. Send bit9 (a0)
  delayUs(1);
  Si4703_SDIO_GPIO_Port->MODER^=(3<<(7*2)); 			//SDIO set as SPI SDIO
  Si4703_SCLK_GPIO_Port->MODER^=(3<<(5*2)); 			//SCLK set as SPI SCLK
  __HAL_SPI_DISABLE(&hspi1);					//Disable SPI. Prepare to change CPOL (polarity of SCLK) to receive data
  hspi1.Instance->CR1^=SPI_CR1_CPOL;				//Changing CPOL
  __HAL_SPI_ENABLE(&hspi1);					//Enable SPI
  HAL_SPI_Receive(&hspi1,(uint8_t*) Si4703_REGs, 2, 1000);	//Read 2 bytes in reverse order
  __HAL_SPI_DISABLE(&hspi1);					//Disable SPI. Prepare to change CPOL (polarity of SCLK) to receive data
  hspi1.Instance->CR1^=SPI_CR1_CPOL;				//Restoring CPOL
  Si4703_SCLK_GPIO_Port->MODER^=(3<<(5*2)); 			//SCLK set as Output
  Si4703_SCLK_GPIO_Port->BSRR|=Si4703_SCLK_Pin<<16;     	//SCLK to 0. Finishing receiving
  Si4703_nSen_GPIO_Port->BSRR|=Si4703_nSen_Pin; 		//nSen to 1. Stop
  Si4703_SCLK_GPIO_Port->MODER^=(3<<(5*2)); 			//SCLK set as SPI SCLK
  return (Si4703_REGs[0]<<8)|Si4703_REGs[1];			//Swap endian

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
  /* USER CODE BEGIN 2 */
  result[0] = Si4703_SPI_Receive(0x01);
  result[1] = Si4703_SPI_Receive(0x00);
  result[1] = Si4703_SPI_Receive(0x01);
  result[1] = Si4703_SPI_Receive(0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
 Si4703_SPI_Init();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Si4703_nReset_Pin|Si4703_nSen_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Si4703_GPIO1_Pin|Si4703_GPIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Si4703_SCLKB12_Pin */
  GPIO_InitStruct.Pin = Si4703_SCLKB12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(Si4703_SCLKB12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Si4703_nReset_Pin Si4703_nSen_Pin Si4703_GPIO1_Pin Si4703_GPIO2_Pin */
  GPIO_InitStruct.Pin = Si4703_nReset_Pin|Si4703_nSen_Pin|Si4703_GPIO1_Pin|Si4703_GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
