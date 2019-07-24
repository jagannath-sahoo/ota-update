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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define C_UART		&huart2
#define D_UART		&huart6
#define MAX_RX_BUFFER	64

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void print_debug_msg(char *format,...);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 char someData[] = "Hello from bootloader\r\n";
 volatile uint32_t addr = 0x08008000;
 uint8_t	bl_rx_buffer[MAX_RX_BUFFER] = {0};
 uint8_t length = 0;
 uint32_t update_flash_base_address = 0;
 uint32_t base_offset_address = 0;
 SignatureTypeDef signature_sector;
 //Flag for extracting of vector from hex string to update the sector address
 uint32_t flag = 1;					
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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//Read the flash structure from boot sector address
	signature_sector = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_BOOT_ADDRESS;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) || signature_sector.updateAvailable)
	//if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
	{
		print_debug_msg("Bootloader Mode\r\n");
		bootloader_uart();
	}
	else
	{
		print_debug_msg("Executing user app\r\n");
		jump_to_user_app(); 
	}
	
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
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

void bootloader_uart()
{
	TypeDef_Intel_Hex flash_data;
	uint8_t status, ret;
	char checksum_str[3];
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15,GPIO_PIN_RESET);
	
	//Erase Sector address
	flash_erase_sector(signature_sector.sector_no);//Get the flash sector from User Application after auth. from Signature Structure
	//char temp[3] = {0};
	//uint16_t line_count;
	//All the booting related code will be here.
	/*Demo code only for testing*/
	//For Debugging
	HAL_UART_Transmit(C_UART,&response,1,HAL_MAX_DELAY);
	while(1)
	{
		//led status for updating
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_14);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15);
		//HAL_UART_Transmit(C_UART,(uint8_t *)someData,strlen(someData),HAL_MAX_DELAY);
		memset(bl_rx_buffer, 0, MAX_RX_BUFFER);
		//strcpy((char *)bl_rx_buffer,":1080000050040020AD810008338D0008978C0008D3");
		HAL_UART_Receive(C_UART,bl_rx_buffer,UART_REC_BLOCK_SIZE,HAL_MAX_DELAY);
		//HAL_UART_Receive_DMA(C_UART,bl_rx_buffer,MAX_RX_BUFFER
		
		//Line number implementation
		//memcpy(temp,&bl_rx_buffer[1],2);
    //line_count = (uint8_t)strtol(temp, NULL, 16);
		
		ret = parse_data_from_intel_hex(&flash_data,bl_rx_buffer);
		if(ret == 0)
		{
			status = HAL_ERROR;
		}
		else
		{
			switch(flash_data.record_type)
			{
				case EXTENDED_LINEAR_ADDR:
					{
						//Set the staring address
						update_flash_base_address = ((flash_data.data[0] << 8) + flash_data.data[1]) << 16;
						//signature_sector.update = flash_data.addr;
						status = HAL_OK;
						break;
					}
				case INTEL_HEX_DATA:
					{
						//Run only once 1st address of new firmware which provide the offset address 
						//for flash starting address
						if(flag)
						{
							flag = 0;//clear
							base_offset_address = flash_data.addr;
						}
						status = write_data_into_flash(&flash_data, update_flash_base_address);
						break;
					}
				case END_OF_FILE:
					{
						signature_sector = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_BOOT_ADDRESS; 	// Reading Boot status structure from flash
						signature_sector.current = update_flash_base_address + base_offset_address;
						signature_sector.updateAvailable = FALSE;									// Setting Flag of update available
						write_to_flash_word(0, &signature_sector, 5);	
						
						// Reset MCU
						NVIC_SystemReset();
						
						status = HAL_OK;
						break;
					}
				default:
				{
					//Nothing to do
					break;
				}
			}
		}
//		if(flash_data.record_type == INTEL_HEX_DATA)
//		{
//			status = write_data_into_flash(&flash_data, FLASH_SECTOR_2_BASE_ADDRESS);
//		}
		
		//HAL_Delay(1000);
		//HAL_UART_Transmit(C_UART,(uint8_t *)flash_data.length,sizeof(flash_data.length),HAL_MAX_DELAY);
		if(status == HAL_OK)
		{
			//HAL_UART_Transmit(C_UART,&response,1,HAL_MAX_DELAY);
			sprintf(checksum_str,"%02X",flash_data.check_sum);
			checksum_str[2] = response;
			//memcpy(&checksum_str[2],&response,1);
			HAL_UART_Transmit(C_UART,(uint8_t *)checksum_str,3,HAL_MAX_DELAY);
			//HAL_UART_Transmit(C_UART,&response,1,HAL_MAX_DELAY);
		}
		else{
			//HAL_UART_Transmit(C_UART,&response,1,HAL_MAX_DELAY);
			flash_data.check_sum = ~(flash_data.check_sum);
			sprintf(checksum_str,"%02X",flash_data.check_sum);
			checksum_str[2] = response;
			HAL_UART_Transmit(C_UART,(uint8_t *)checksum_str,1,HAL_MAX_DELAY);
			//HAL_UART_Transmit(C_UART,&response,1,HAL_MAX_DELAY);
		}
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
//		uint32_t currentTick = HAL_GetTick();
//		print_debug_msg("Curren Tick: %d\r\n",currentTick);
//    while(HAL_GetTick() <= (currentTick + 500));
	}
}

uint8_t write_data_into_flash(TypeDef_Intel_Hex *handle, uint32_t mem_base_addr)
{
	HAL_FLASH_Unlock();
	uint8_t status;
	//Set starting memory address 
	//handle->addr = handle->addr + mem_base_addr;
	mem_base_addr = mem_base_addr + handle->addr;
	
	for(uint32_t i = 0 ; i < handle->length ; i++)
	{
		//writing into the flash byte by byte
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_base_addr + i,handle->data[i] );
		
		//Error handling
		if(status != HAL_OK)
		{
			HAL_FLASH_Lock();
			return HAL_ERROR;
		}
	}
  HAL_FLASH_Lock();
	return status;
}

void jump_to_user_app()
{
	//Ptr to hold the address of reset handler
	void (*user_app_reset_handler)(void);
	print_debug_msg("BL: Jump to USER APP\r\n");
	
	//Read the MSP value from the BASE address of Flash
	//uint32_t msp_address = *(volatile uint32_t *)FLASH_SECTOR_2_BASE_ADDRESS;
	uint32_t msp_address = *(volatile uint32_t *)signature_sector.current;
	print_debug_msg("BL: APP MSP Address: %#x",msp_address);
	__set_MSP(msp_address);
	
	 //uint32_t msp_value = *((volatile uint32_t*)(FLASH_SECTOR_2_BASE_ADDRESS));
   //print_debug_msg("BL_DEBUG_MSG:MSP value : %#x\r\n",addr);
	 //This function comes from CMSIS.
   //__set_MSP(*(uint32_t *)addr);

	
	//Fetching of address of reset handler from Location
	//FLASH_SECTOR_2_BASE_ADDRESS + 4
	//Then Jump to USER APP
	//uint32_t resethanlder_address = *(volatile uint32_t*)(FLASH_SECTOR_2_BASE_ADDRESS + 4);
	uint32_t resethanlder_address = *(volatile uint32_t*)(signature_sector.current + 4);
	print_debug_msg("BL: APP Reset Handler Address:%#x \r\n",resethanlder_address);
	user_app_reset_handler = (void(*)(void))resethanlder_address;
	//app_reset_handler();
	
	//app_reset_handler = (void (*)(void)) (*((uint32_t *)(addr + 4)));
	user_app_reset_handler();
}

uint8_t parse_data_from_intel_hex(TypeDef_Intel_Hex *handle, uint8_t *rx_buffer)
{
    char temp[3] = {0};
    //Defines whether is Address or flash_data
    //uint8_t hex_type = 0;
    
    //Length of Intel Hex
    memcpy(temp,&rx_buffer[1],2);
    handle->length = (uint8_t)strtol(temp, NULL, 16);
    
    //Address from Intel Hex
    memset(temp,0,3);
    memcpy(temp,&rx_buffer[3],4);
    handle->addr = (uint32_t)strtol(temp, NULL, 16);

    //Record type
    memset(temp,0,3);
    memcpy(temp,&rx_buffer[7],2);
    handle->record_type = (uint8_t)strtol(temp, NULL, 16);
    
    //data type
    uint8_t j = 9;
    for (int i = 0; i < handle->length; i++) {
        memset(temp,0,3);
        memcpy(temp,&rx_buffer[j],2);
        handle->data[i] = (uint8_t)strtol(temp, NULL, 16);
        j+=2;
    }
    
    //check sum
    memset(temp,0,3);
    memcpy(temp,&rx_buffer[j],2);
    handle->check_sum = (uint8_t)strtol(temp, NULL, 16);
    
    //calculate check sum
    uint8_t check_sum_cal = 0;
    check_sum_cal = handle->length;// + handle->addr + handle->record_type;
    check_sum_cal = check_sum_cal + (handle->addr >> 8);
    check_sum_cal = check_sum_cal + (handle->addr & 0x00ff);
    check_sum_cal = check_sum_cal + handle->record_type;
    for (int i = 0; i < handle->length; i++) {
      check_sum_cal = check_sum_cal + handle->data[i];
    }
    check_sum_cal = 0x01 + ~(check_sum_cal);
    //printf("%s\n", (handle->check_sum == check_sum_cal)? "MATCH":"FAILED");
    return ((handle->check_sum == check_sum_cal)? 1 : 0);
}

void flash_erase_sector(uint32_t sector_no)
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(sector_no, FLASH_VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}


void write_to_flash_word(uint32_t offSet, void *wrBuf, uint32_t Nsize)
{
	uint32_t flashAddress = SIGNATURE_BOOT_ADDRESS + offSet;
	
	//Erase sector before write
	flash_erase_sector(SIGNATURE_SECTOR_NO);
	
	//Unlock Flash
	HAL_FLASH_Unlock();
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress + 8, 0xFFFFFFFF);
	//Write to Flash
	for(uint32_t i=0; i<Nsize; i++)
	{
		//flag = ((uint32_t *)wrBuf)[i];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , ((uint32_t *)wrBuf)[i]);
		flashAddress+=4;
	}
	
	//Lock the Flash space
	HAL_FLASH_Lock();
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
