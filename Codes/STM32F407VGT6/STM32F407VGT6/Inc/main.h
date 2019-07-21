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
	BootSector update;				// Address of sector for the update
	uint32_t updateAvailable;	// To store the status of update
} BootSectorTypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TRUE	1
#define FALSE	0
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
void write_to_flash(uint32_t offSet, void *wrBuf, uint32_t Nsize);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEMS_CS_Pin GPIO_PIN_3
#define MEMS_CS_GPIO_Port GPIOE
#define SWITCH1_Pin GPIO_PIN_0
#define SWITCH1_GPIO_Port GPIOA
#define SWITCH1_EXTI_IRQn EXTI0_IRQn
#define UPDATE_AVAIL_Pin GPIO_PIN_1
#define UPDATE_AVAIL_GPIO_Port GPIOB
#define UPDATE_AVAIL_EXTI_IRQn EXTI1_IRQn
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
