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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef uint32_t BootSector;
static uint8_t response = '$';

// Structure to be stored in last segment of flash
// to select boot sector for booting up and update
typedef struct{
	BootSector current;				// Address of current code sector
	uint32_t updateAvailable;	// To store the status of update
	uint32_t sector_no;				// Sector No of the sector to be updated
	uint32_t sector_addr;			// Sector Address of the Sector to be updated
	uint32_t version;					// Version of the Firmware
} SignatureTypeDef;					

// OTA update request structure
// This Structure is used to store the Authentication Data sent by the ESP
// if the new update is available
typedef struct{
	char board[32];
	uint32_t version;
	char secure_id[10];
	uint32_t sector_no;
	uint32_t sector_addr;
}OTAUpdateRequestTypeDef;

// This structure will store X,Y,Z values read from Accelerometer
// in raw format
typedef struct{
	int16_t xVal;
	int16_t yVal;
	int16_t zVal;
} AccelDataTypeDef;

// This structure will store the values read from Accelerometer
// in Scaled Format
typedef struct{
	float xVal;
	float yVal;
	float zVal;
} AccelScaledDataTypeDef;

// This structure denotes the one block of the message queue
typedef struct Message{
	char data[32];
	//uint8_t idx;
}MessageQueue_st;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define TRUE	1
#define FALSE	0

#define MAX_RX_BUFFER 64

#define SEC 			1000
#define MINUTE		(60*SEC)

#define CHECKUPDATE "check_update"

//AUTHORIZATION INFORMATION
#define BOARD_NAME	"STM32F407VGT6"
#define SECURE_ID		"A1B2C3D4"

#define STM32F407VGT6
#ifdef STM32F407VGT6
	#define SIGNATURE_SECTOR_NO 			11
	#define SIGNATURE_FLASH_ADDRESS 	0x080E0000
#else
	#define SIGNATURE_SECTOR_NO 			7
	#define SIGNATURE_FLASH_ADDRESS 	0x08060000
#endif

#ifdef STM32F407VGT6
//SPI Chip Select
#define _CS_ENABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define _CS_DISABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#endif

// Addresses of X, Y, Z registers in the Accelerometer
#define OUT_X_ADDR				0x28
#define OUT_Y_ADDR        0x2A
#define OUT_Z_ADDR        0x2C

#define SENSITIVITY				0.06  /* 0.06 mg/digit*/

// Setting up the Flag Mask for the Event Flags
#define FLAGS_MSK			0x00000003U


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
  * @brief Erasing the provided flash sector
  * @param void
  * @retval void
  */
static void FLASH_EraseSector(void);

/**
  * @brief Writing data to flash memory
  * @param offSet		:		offset from the starting address of the sector
	* @param wrBf			: 	pointer to the buffer from which data is to be written
	*	@param Nsize		:		Number of data blocks to be wriien
  * @retval void
  */
void write_to_flash_word(uint32_t offSet, void *wrBuf, uint32_t Nsize);

/**
  * @brief Parsing upadte request string from ESP
  * @param handle reference of OTAUpdateRequestTypeDef for parsing
	* @param data	update requested data receied from ESP
  * @retval pointer to OTAUpdateRequestTypeDef type handle
  */
OTAUpdateRequestTypeDef* parsing_update_request(OTAUpdateRequestTypeDef *handle, char *data);

/**
  * @brief Parsing upadte request string from ESP
  * @param str string to reverse
  * @retval pointer to reversed string
  */
char *strrev(char *str);

#ifdef STM32F407VGT6
/**
  * @brief Initializing the Accelerometer parameters
  * @param SPI handle to which Accelerometer is connected
  * @retval void
  */
void accel_init(SPI_HandleTypeDef *);

/**
  * @brief Reading X,Y,Z values from the Accelerometer
  * @param void
  * @retval Structure to store the read X,Y,Z values
  */
AccelDataTypeDef accel_data_read(void);

/**
  * @brief 	Reading X,Y,Z values from the Accelerometer
						in a scaled format using sensitivity
  * @param 	void
  * @retval Structure to store the read X,Y,Z scaled values
  */
AccelScaledDataTypeDef get_scaled_data(void);

/**
  * @brief Reading registers from the Accelerometer
  * @param register to be read
	* @param RX Buufer to store the read value
	* @param No of Bytes to be read
  * @retval void
  */
void	readIO(uint8_t reg, uint8_t *dataR , uint8_t size);

/**
  * @brief Writing value to the Register of accelerometer
  * @param register to which data is to be written
	* @param Data to be wriiten
	* @param No of bytes to be written
  * @retval void
  */  
void writeIO(uint8_t reg, uint8_t *dataW, uint8_t size);
#endif

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEMS_CS_Pin GPIO_PIN_3
#define MEMS_CS_GPIO_Port GPIOE
#define LED_Green_Pin GPIO_PIN_12
#define LED_Green_GPIO_Port GPIOD
#define LED_Orange_Pin GPIO_PIN_13
#define LED_Orange_GPIO_Port GPIOD
#define LED_Red_Pin GPIO_PIN_14
#define LED_Red_GPIO_Port GPIOD
#define LED_Blue_Pin GPIO_PIN_15
#define LED_Blue_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
