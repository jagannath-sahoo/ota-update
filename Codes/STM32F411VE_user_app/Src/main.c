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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBEG_MSG_EN

#define C_UART		&huart2
#define D_UART		&huart6

#define MAX_RX_BUFFER	64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void print_debug_msg(char *format,...);
void ISR_buttom_half();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SignatureTypeDef signature_sector;
uint8_t	rx_buffer[MAX_RX_BUFFER] = {0};
uint32_t flag;					// For testing
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//bootSectorStatus.current 					= 0x08008000;	//sector 2
	//bootSectorStatus.update						= 0x08010000;	//sector 4
	//bootSectorStatus.updateAvailable	=	FALSE;
	
	//write_to_flash(0, &bootSectorStatus,3);
	
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
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	//char someData[] = "Hello from USER APP\r\n";
  /* USER CODE END 2 */
	//FLASH_EraseSector();				
	signature_sector = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_FLASH_ADDRESS;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//		signature_sector.sector_no = 2;
//		signature_sector.starting_address = 0x08008000;
//		signature_sector.updateAvailable = 1;
//		write_to_flash_word(0,&signature_sector,4);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
		HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

static void print_debug_msg(char *format,...)
{
	#ifdef BL_DEBEG_MSG_EN
	char str[80];
	
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
	va_end(args);
	#endif
}

static void FLASH_EraseSector(void)
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(SIGNATURE_SECTOR_NO, FLASH_VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}


void write_to_flash_word(uint32_t offSet, void *wrBuf, uint32_t Nsize)
{
	uint32_t flash_address = SIGNATURE_FLASH_ADDRESS + offSet;
	
	//Erase sector before write
	FLASH_EraseSector();				// This will increasing the latency by substancial amount
	
	//Unlock Flash
	HAL_FLASH_Unlock();
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress + 8, 0xFFFFFFFF);
	//Write to Flash
	for(uint32_t i=0; i<Nsize; i++)
	{
		//flag = ((uint32_t *)wrBuf)[i];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address , ((uint32_t *)wrBuf)[i]);
		//HAL_Delay(2000);
		flash_address+=4;
	} 
	
	//Lock the Flash space
	HAL_FLASH_Lock();
}


//Defining callback for the external interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	
	if(GPIO_Pin == GPIO_PIN_1)//Original GPIO_PIN_0
	{
		// Update Switch Pressed
		signature_sector = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_FLASH_ADDRESS;	// Reading Boot status structure from flash
		if(signature_sector.updateAvailable == TRUE)
		{
			//JUMP TO BOOTLOADER
			print_debug_msg("Bootloader JUMP\r\n");
			
			// Reset MCU
			NVIC_SystemReset();
			//while(1)
			//{
			//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
			//	HAL_Delay(200);
			//}
			//{ JUMP CODE TO BE WRITTEN}
			// DEBOUNCING DELAY
		}
	}
	

	if(GPIO_Pin == GPIO_PIN_0)//Original GPIO_PIN_1
	{
		//HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		ISR_buttom_half();
		//HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	}
}

OTAUpdateRequestTypeDef* parsing_update_request(OTAUpdateRequestTypeDef *handle, char *data)
{ 
	char* token = strtok(data, "$"); 
	strcpy(handle->board, token);
	
	token = strtok(NULL, "$");
	handle->version = (uint32_t)strtol(token, NULL, 10);
	
	token = strtok(NULL, "$");
	strcpy(handle->secure_id, token);
	
	token = strtok(NULL, "$");
	handle->sector_no = (uint32_t)strtol(token, NULL, 10);
	
	token = strtok(NULL, "$");
	handle->sector_addr = (uint32_t)strtol(token, NULL, 16);
	
	return handle;
}


char *strrev(char *str)
{
	char c, *front, *back;
	if(!str || !*str)
			return str;
	for(front=str,back=str+strlen(str)-1;front < back;front++,back--){
			c=*front;*front=*back;*back=c;
	}
	return str;
}

void ISR_buttom_half()
{
			// Update is available processing event
		//Create a buffer for receive on uart
		OTAUpdateRequestTypeDef update_request;
		//SignatureTypeDef temp;
		uint8_t flag = 0;

		signature_sector = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_FLASH_ADDRESS; 	// Reading Boot status structure from flash
		//uint32_t current = signature_sector.current;
		
		memset(rx_buffer, 0, MAX_RX_BUFFER);
		//1. 1st response from stm32 to esp as it has ack for update structure
		HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
		//Prepare the buffer for receiving update information
		HAL_UART_Receive(C_UART, rx_buffer, MAX_RX_BUFFER, HAL_MAX_DELAY);
		
		parsing_update_request(&update_request, (char *)rx_buffer);
		
		if((strcmp(BOARD_NAME, update_request.board) == 0) 
					&& (strcmp(SECURE_ID, update_request.secure_id) == 0)
					/*&& (strcmp(signature_sector.version,update_request.version) == 0)*/)	
		{
			flag = 1;
		}
		else
		{
			flag = 0;
		}
		
		if(flag == 1)
		{
			//AUTH Success
			//signature_sector.current = signature_sector.starting_address;
			signature_sector.sector_no = update_request.sector_no;
			signature_sector.sector_addr = update_request.sector_addr;
			signature_sector.version = update_request.version;
			signature_sector.updateAvailable = TRUE;									// Setting Flag of update available
			
			write_to_flash_word(0, &signature_sector, 5);										// Writing the updated structure to flash
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);			// Making Green LED ON as a status of available update
		//bootSectorStatus.updateAvailable = FALSE;	// For TESTING
			//Response to the ESP
			//temp = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_FLASH_ADDRESS; 	
			strrev(update_request.secure_id);//Reverse of secure id
			HAL_UART_Transmit(C_UART,(uint8_t *)update_request.secure_id, strlen(update_request.secure_id),HAL_MAX_DELAY);
			HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
			return;
		}
		else
		{
			//Failed AUTH
			//Return same secure id which was received
			HAL_UART_Transmit(C_UART,(uint8_t *)update_request.secure_id, strlen(update_request.secure_id),HAL_MAX_DELAY);
			HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
		}
}
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
