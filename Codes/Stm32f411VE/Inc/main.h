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

// Structure to be stored in last segment of flash
// to select boot sector for booting up and update
typedef struct{
	BootSector current;				// Address of current code sector
	uint32_t updateAvailable;	// To store the status of update
	uint32_t sector_no;
	uint32_t sector_addr;
	uint32_t version;
} SignatureTypeDef;

typedef struct Intel_Hex{
  uint8_t length;
  uint32_t addr;
  uint8_t record_type;
  uint8_t data[32];
  uint8_t check_sum;
}TypeDef_Intel_Hex;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//For debugging
static uint8_t response = '$';
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define TRUE	1
#define FALSE	0

#define	INTEL_HEX_DATA  			0x00
#define	END_OF_FILE 					0x01
#define	EXTENDED_SEGMENT_ADDR 0x02
#define	EXTENDED_LINEAR_ADDR 	0x04
#define	START_LINEAR_ADDR 		0x05

#define UART_REC_BLOCK_SIZE		64

//#define STM32F407VGT6
#ifdef STM32F407VGT6
	#define SIGNATURE_SECTOR_NO 			11
	#define SIGNATURE_BOOT_ADDRESS 	0x080E0000
#else
	#define SIGNATURE_SECTOR_NO 			7
	#define SIGNATURE_BOOT_ADDRESS 	0x08060000
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
  * @brief Jumping the execution to the bootloader application
  * @param void
  * @retval void
  */
void bootloader_uart(void);

/**
  * @brief Jumping the execution to the user application sector address
  * @param void
  * @retval void
  */
void jump_to_user_app(void);

/**
  * @brief Parsing data from Intel Hex formated string
  * @param  handle Pointer to the Intel Hex string struct TypeDef_Intel_Hex get_flash_data
  *         contains intel hex file struct
  * @param  rx_buffer Pointer to RX data buffer
  * @retval status if check sum faild then returns 0, if successful then
  *         returns 1
  */
uint8_t parse_data_from_intel_hex(TypeDef_Intel_Hex *handle, uint8_t *rx_buffer);

/**
  * @brief Writting data into the flash in byte by byte order as provided with Intel Hex String
  * @param TypeDef_Intel_Hex type structure contains the Intel Hex string information in 
					 sorted order
	* @param Base address as flash sector memory address where to write
  * @retval if successful returns HAL_OK else HAL_ERROR
  */
	
uint8_t write_data_into_flash(TypeDef_Intel_Hex *handle, uint32_t mem_base_addr);

/**
  * @brief Erasing the provided flash sector
  * @param sector no for clearing sector address
  * @retval void
  */
void flash_erase_sector(uint32_t sector_no);

/**
  * @brief Writing data to flash memory
  * @param offSet offset from the starting address of the sector
	* @param wrBf	pointer to the buffer from which data is to be written
	*	@param Nsize Number of data blocks to be wriien
  * @retval void
  */
void write_to_flash_word(uint32_t offSet, void *wrBuf, uint32_t Nsize);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR_2_BASE_ADDRESS		0x08008000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
