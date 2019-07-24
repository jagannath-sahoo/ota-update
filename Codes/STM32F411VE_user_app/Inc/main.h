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
	uint32_t sector_no;
	uint32_t sector_addr;
	uint32_t version;
} SignatureTypeDef;

//OTA update request structure
typedef struct{
	char board[32];
	uint32_t version;
	char secure_id[10];
	uint32_t sector_no;
	uint32_t sector_addr;
}OTAUpdateRequestTypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TRUE	1
#define FALSE	0

#define MAX_RX_BUFFER 64

//AUTHORIZATION INFORMATION
#define BOARD_NAME	"STM32F407VGT6"
#define SECURE_ID		"A1B2C3D4"

//#define STM32F407VGT6
#ifdef STM32F407VGT6
	#define SIGNATURE_SECTOR_NO 			11
	#define SIGNATURE_FLASH_ADDRESS 	0x080E0000
#else
	#define SIGNATURE_SECTOR_NO 			7
	#define SIGNATURE_FLASH_ADDRESS 	0x08060000
#endif

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
