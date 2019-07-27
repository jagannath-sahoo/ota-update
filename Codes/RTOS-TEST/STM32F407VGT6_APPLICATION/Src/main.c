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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBEG_MSG_EN			// For Debugging
#define C_UART		&huart2
#define D_UART		&huart6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId_t accelTaskHandle;
osThreadId_t sendTaskHandle;
osThreadId_t updateTaskHandle;
osThreadId_t checkUpdateHandle;
osMessageQueueId_t accelDataQueueHandle;
osMutexId_t uartMutexHandle;
/* USER CODE BEGIN PV */

#ifdef STM32F407VGT6
uint8_t spiTxBuf[2];		// SPI Transmit Buffer
uint8_t spiRxBuf[2];		// SPI Receive Buffer
uint8_t reg;						// register of Accelerometer
AccelDataTypeDef myAccelData;
AccelScaledDataTypeDef myAccelScaledData;
#endif

char uartTxBuf[30];
osEventFlagsId_t evt_id;
uint32_t flags;
osStatus_t status;

//----------------------------------------------------
SignatureTypeDef signature_sector;
uint8_t	rx_buffer[MAX_RX_BUFFER] = {0};
//----------------------------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void AccelReadThread(void *argument);
void TransmitUartThread(void *argument);
void UpdateHandleThread(void *argument);
void CheckUpdateThread(void *argument);

/* USER CODE BEGIN PFP */
static void print_debug_msg(char *format,...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
	
  /* USER CODE BEGIN 2 */
	#ifdef STM32F407VGT6
	accel_init(&hspi1);	// initializing Accelerometer
	#endif
  /* USER CODE END 2 */

  osKernelInitialize();

  /* Create the mutex(es) */
  /* definition and creation of uartMutex */
  const osMutexAttr_t uartMutex_attributes = {
    .name = "uartMutex"
  };
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of accelDataQueue */
  const osMessageQueueAttr_t accelDataQueue_attributes = {
    .name = "accelDataQueue"
  };
  accelDataQueueHandle = osMessageQueueNew (4, sizeof(MessageQueue_st), &accelDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of accelTask */
  const osThreadAttr_t accelTask_attributes = {
    .name = "accelTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 480
  };
  accelTaskHandle = osThreadNew(AccelReadThread, NULL, &accelTask_attributes);

  /* definition and creation of sendTask */
  const osThreadAttr_t sendTask_attributes = {
    .name = "sendTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 240
  };
  sendTaskHandle = osThreadNew(TransmitUartThread, NULL, &sendTask_attributes);

  /* definition and creation of updateTask */
  const osThreadAttr_t updateTask_attributes = {
    .name = "updateTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 480
  };
  updateTaskHandle = osThreadNew(UpdateHandleThread, NULL, &updateTask_attributes);

  /* definition and creation of checkUpdate */
  const osThreadAttr_t checkUpdate_attributes = {
    .name = "checkUpdate",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  checkUpdateHandle = osThreadNew(CheckUpdateThread, NULL, &checkUpdate_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	//  creates a new event flags object that is used to 
	//	send events across thread
	//	It can be safely called before the RTOS is started
	//	(call to osKernelStart), but not before it is initialized 
	//	(call to osKernelInitialize).
	evt_id = osEventFlagsNew(NULL);
	  if (evt_id == NULL) {
    ; // Event Flags object not created, handle failure
    while(1);
  }
	osEventFlagsClear(evt_id, FLAGS_MSK);
	
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_Green_Pin|LED_Orange_Pin|LED_Red_Pin|LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MEMS_CS_Pin */
  GPIO_InitStruct.Pin = MEMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MEMS_CS_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : LED_Green_Pin LED_Orange_Pin LED_Red_Pin LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin|LED_Orange_Pin|LED_Red_Pin|LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
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

#ifdef STM32F407VGT6
// Accelerometer Initializtion function
void accel_init(SPI_HandleTypeDef *hspi)
{
	/*	Default Full Scale = 2g
	 *	For this Default Full Scale value, Sensitivity = 0.06 mg/digit
	 *	If the device is perfectly level and not moving, then:
					----> X/Y accel axes should read 0
					---->	Z accel axis should read 1g, 
								which is +16384 at a sensitivity of 2g
	*/
	
	reg = 0x20;									// CTRL_REG4
	spiTxBuf[0]	=	0x17;					/*Enabling X, Y and z axis
																Setting Data Rate to 3.125 Hz */
	writeIO(reg,spiTxBuf, 1);		
	
	reg = 0x25;									//CTRL_REG6
	spiTxBuf[0]	=	0x10;					//AUTO INC
	writeIO(reg,spiTxBuf, 1);
}


// Reading Accelerometer RAW values 
AccelDataTypeDef accel_data_read(void)
{
	AccelDataTypeDef tempData;
	
	//Reading X value
	readIO(OUT_X_ADDR, spiRxBuf, 2);
	tempData.xVal = ((spiRxBuf[1] << 8) + spiRxBuf[0]);
	
	//Reading Y value
	readIO(OUT_Y_ADDR, spiRxBuf, 2);
	tempData.yVal = ((spiRxBuf[1] << 8) + spiRxBuf[0]);
	
	//Reading Z value
	readIO(OUT_Z_ADDR, spiRxBuf, 2);
	tempData.zVal = ((spiRxBuf[1] << 8) + spiRxBuf[0]);
	
	return tempData;
}

// Reading X,Y,Z in scaled format
AccelScaledDataTypeDef get_scaled_data(void)
{
	AccelDataTypeDef tempRawData =  accel_data_read();
	AccelScaledDataTypeDef tempScaledData;
	
	tempScaledData.xVal = (tempRawData.xVal * SENSITIVITY) + 0.0f;
	tempScaledData.yVal = (tempRawData.yVal * SENSITIVITY) + 0.0f;
	tempScaledData.zVal = (tempRawData.zVal * SENSITIVITY) + 0.0f;
	
	return tempScaledData;
}

void writeIO(uint8_t reg, uint8_t *dataW, uint8_t size)
{
	/* 	For TRANSMITTING DATA via SPI following 
			3 Steps must be followed:
	1. Bringing Slave Select line low
	2. Tranmitting Register + data
	3. Bringing Slave Select to high
	*/
	
	//Enable CS
	_CS_ENABLE;
	//set register value
	HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
	//Transmit data
	HAL_SPI_Transmit(&hspi1, dataW, size, 10);
	_CS_DISABLE;
}

void	readIO(uint8_t reg, uint8_t *dataR, uint8_t size)
{
	/* 	For RECEIVING DATA from the Accelerometer 
			following 4 steps must be followed:
	1. Bringing Slave Select line low
	2. Tranmitting Register +0x80
	3. Receiving Data
	4. Bringing Slave Select to high
	*/
	
	uint8_t temp = reg|0x80;	/*0x80 makes MSB 1 denoting that Master needs 
															to read data from the slave (Accelerometer)*/
	//Enable CS
	_CS_ENABLE;
	//set register value
	HAL_SPI_Transmit(&hspi1, &temp, 1, 10);
	//Transmit data
	HAL_SPI_Receive(&hspi1, dataR, size, 10);
	//Disable CS
	_CS_DISABLE;
}
#endif /* STM32F407VGT6 */

//Defining callback for the external interrupt
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	if(GPIO_Pin == GPIO_PIN_0)
	{
		// Update Switch Pressed
		signature_sector = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_FLASH_ADDRESS;	// Reading Boot status structure from flash
		if(signature_sector.updateAvailable == TRUE)
		{
			// Authentic Update is Available
			osEventFlagsSet(evt_id, 0x00000002U);
			// for(uint32_t i=0; i<10000;i++);						// DEBOUNCING DELAY
		}
	}
	
	if(GPIO_Pin == GPIO_PIN_1)
	{
		// Update is available
		//bootSectorStatus = *(BootSectorTypeDef *)(uint32_t *)SECTOR_ADDRESS; 	// Reading Boot status structure from flash
		//bootSectorStatus.updateAvailable = TRUE;									// Setting Flag of update available
		//write_to_flash(0, &bootSectorStatus,3);										// Writing the updated structure to flash
		osEventFlagsSet(evt_id, 0x00000001U);
		//bootSectorStatus.updateAvailable = FALSE;	// For TESTING
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


/* USER CODE END 4 */

/* USER CODE BEGIN Header_AccelReadThread */
/**
  * @brief  Function implementing the accelTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_AccelReadThread */
void AccelReadThread(void *argument)
{
    
    
    
    
    
    

  /* USER CODE BEGIN 5 */
	float x,y,z;
	MessageQueue_st msg;
	osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
		#ifdef STM32F407VGT6
		//myAccelData = accel_data_read();
		myAccelScaledData = get_scaled_data();
		x = myAccelScaledData.xVal;
		y = myAccelScaledData.yVal;
		z = myAccelScaledData.zVal;
		sprintf(uartTxBuf,"%04.3f,%04.3f,%04.3f\n\r",x,y,z);
		#else
		sprintf(uartTxBuf,"Hello From STM32F411",);
		#endif
		strcpy(msg.data,uartTxBuf);//Copy msg to the buffer
		
		
		//p1: handle to message queue
		//p2: address to the message structure
		//p3: Priority is NULL
		//p4: No wait returns instantly
		/*
			case 1: p4 as osWaitForever
							Queue was blocked untill msg delivered
			case 2: p4 as no wait still it's working
			*/
		status = osMessageQueuePut(accelDataQueueHandle, &msg, NULL,0);
		if(status != osOK)
		{
			//Error Handling
			HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
		}
		//HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuf,strlen(uartTxBuf),50);
    osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_TransmitUartThread */
/**
* @brief Function implementing the sendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmitUartThread */
void TransmitUartThread(void *argument)
{
  /* USER CODE BEGIN TransmitUartThread */
	MessageQueue_st recMsg;
	
  /* Infinite loop */
  for(;;)
  {
		//p1: handle to message queue
		//p2: address to the message structure
		//p3: Priority is NULL
		//p4: No wait returns instantly or osWaitForever
		//		In both the cases it working
		status = osMessageQueueGet(accelDataQueueHandle, &recMsg, NULL, HAL_MAX_DELAY);
		if(status == osOK)
		{
			//Success
			
			//---- APPLY MUTEX HERE(ACQUIRE)
			if (uartMutexHandle != NULL) {
				status = osMutexAcquire(uartMutexHandle, osWaitForever);
				if (status != osOK)  {
					// handle failure code
					HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
				}
				else
				{
					//----------------------- CRITICAL SECTION ----------------------------------------
					HAL_UART_Transmit(C_UART,(uint8_t *)recMsg.data,strlen(recMsg.data), HAL_MAX_DELAY);
					//-----------------------------------------------------------------------------------
				}
				//---- APPLY MUTEX HERE(RELEASE)
				status = osMutexRelease(uartMutexHandle);
				if (status != osOK)  {
					// handle failure code
					HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
				}
			}
			memset(recMsg.data,0,32);	
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
		}
    osDelay(1);
  }
  /* USER CODE END TransmitUartThread */
}

/* USER CODE BEGIN Header_UpdateHandleThread */
/**
* @brief Function implementing the updateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UpdateHandleThread */
void UpdateHandleThread(void *argument)
{
  /* USER CODE BEGIN UpdateHandleThread */
  /* Infinite loop */
  for(;;)
  {
		// Create a buffer for receive on uart
		OTAUpdateRequestTypeDef update_request;
		uint8_t updateFlag = 0;
		flags = 0;
		flags = osEventFlagsWait(evt_id, FLAGS_MSK, osFlagsWaitAny, osWaitForever);
		if(flags != osFlagsErrorResource && flags != osFlagsErrorParameter)
		{
			if((flags&0x00000001) == 0x00000001)
			{
				// Update is Available
				
				//Prepare the buffer for receiving update information
				memset(rx_buffer, 0, MAX_RX_BUFFER);
				
				// Send '$' token to the ESP to ask for Authentication Frame
				// 1st response from stm32 to ESP as it has ack for update structure
				
				//---- APPLY MUTEX HERE(ACQUIRE)
				if (uartMutexHandle != NULL) {
					status = osMutexAcquire(uartMutexHandle, osWaitForever);
					if (status != osOK)  {
						// handle failure code
						HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
					}
					else
					{
						//----------------------- CRITICAL SECTION ----------------------------
						HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
				
						// When '$' is received, RPi will send the AUTHORIZATION FRAME to STM.
						HAL_UART_Receive(C_UART, rx_buffer, MAX_RX_BUFFER, HAL_MAX_DELAY);
						//---------------------------------------------------------------------
					}
					//---- APPLY MUTEX HERE(RELEASE)
					status = osMutexRelease(uartMutexHandle);
					if (status != osOK)  {
						// handle failure code
						HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
					}
				}
				
				
				/* Check for Authentication of update
				// If it is Authentic, update the Signature Structure with updateFlag = TRUE
				// Also, turn ON Green LED to notify user about the update and also send ACK
				// to the ESP32 which is nothing but the but just the SecureID in Reverse order*/
				parsing_update_request(&update_request, (char *)rx_buffer);
				
				if((strcmp(BOARD_NAME, update_request.board) == 0) 
					&& (strcmp(SECURE_ID, update_request.secure_id) == 0)
					/*&& (strcmp(signature_sector.version,update_request.version) == 0)*/)	
				{
					updateFlag = 1;
				}
				else
				{
					updateFlag = 0;
				}
				
				if(updateFlag == 1)
				{
					//AUTH Success
					//signature_sector.current = signature_sector.starting_address;
					signature_sector.sector_no = update_request.sector_no;
					signature_sector.sector_addr = update_request.sector_addr;
					signature_sector.version = update_request.version;
					signature_sector.updateAvailable = TRUE;				// Setting Flag of update available
			
					write_to_flash_word(0, &signature_sector, 5);		// Writing the updated structure to flash

					//Response (ACK) to the ESP
					//temp = *(SignatureTypeDef *)(uint32_t *)SIGNATURE_FLASH_ADDRESS; 	
					strrev(update_request.secure_id);								//Reverse of secure id
					
					//---- APPLY MUTEX HERE(ACQUIRE)
					if (uartMutexHandle != NULL) {
						status = osMutexAcquire(uartMutexHandle, osWaitForever);
						if (status != osOK)  {
							// handle failure code
							HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
						}
						else
						{
							//------------------------------------- CRITICAL SECTION ---------------------------------------------------
							HAL_UART_Transmit(C_UART,(uint8_t *)update_request.secure_id, strlen(update_request.secure_id),HAL_MAX_DELAY);
							HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
							//------------------------------------------------------------------------------------------------------------
						}
						//---- APPLY MUTEX HERE(RELEASE)
						status = osMutexRelease(uartMutexHandle);
						if (status != osOK)  {
							// handle failure code
							HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
						}
					}				
					
					HAL_GPIO_WritePin(GPIOD, LED_Green_Pin, GPIO_PIN_SET);				// Making Green LED ON as a status of available update
					return;
				}
				else
				{
					//Failed AUTH
					//Return same secure id which was received (NACK)
					//---- APPLY MUTEX HERE(RELEASE)
					if (uartMutexHandle != NULL) {
						status = osMutexAcquire(uartMutexHandle, osWaitForever);
						if (status != osOK)  {
							// handle failure code
							HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
						}
						else
						{
							//------------------------------------- CRITICAL SECTION ---------------------------------------------------
							HAL_UART_Transmit(C_UART,(uint8_t *)update_request.secure_id, strlen(update_request.secure_id),HAL_MAX_DELAY);
							HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
							//------------------------------------------------------------------------------------------------------------
						}
						//---- APPLY MUTEX HERE(RELEASE)
						status = osMutexRelease(uartMutexHandle);
						if (status != osOK)  {
							// handle failure code
							HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
						}
					}
				}		
			}
			else if((flags&0x00000002) == 0x00000002)
			{
				// Authentic Update is Available
				HAL_GPIO_TogglePin(GPIOD, LED_Blue_Pin);
				//JUMP TO BOOTLOADER
				print_debug_msg("Bootloader JUMP\r\n");
			
				// Reset MCU
				NVIC_SystemReset();
			}
		}
		osDelay(1);
	}
  /* USER CODE END UpdateHandleThread */
}

/* USER CODE BEGIN Header_CheckUpdateThread */
/**
* @brief Function implementing the checkUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CheckUpdateThread */
void CheckUpdateThread(void *argument)
{
  /* USER CODE BEGIN CheckUpdateThread */
  /* Infinite loop */
  for(;;)
  {
		//---- APPLY MUTEX HERE(ACQUIRE)
		if (uartMutexHandle != NULL) {
			status = osMutexAcquire(uartMutexHandle, osWaitForever);
			if (status != osOK)  {
				// handle failure code
				HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
			}
			else
			{
				//------------------------------------- CRITICAL SECTION -------------------------
				HAL_UART_Transmit(C_UART,(uint8_t *)"CHECK", strlen("CHECK"),HAL_MAX_DELAY);
				HAL_UART_Transmit(C_UART, &response, 1, HAL_MAX_DELAY);
				//-------------------------------------------------------------------------------
			}
			//---- APPLY MUTEX HERE(RELEASE)
			status = osMutexRelease(uartMutexHandle);
			if (status != osOK)  {
				// handle failure code
				HAL_GPIO_WritePin(GPIOD, LED_Red_Pin, GPIO_PIN_SET);
			}
		}
		
    osDelay(2*MINUTE);
  }
  /* USER CODE END CheckUpdateThread */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
