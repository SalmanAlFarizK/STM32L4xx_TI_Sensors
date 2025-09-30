/******************************************************************************
 *		Header File Containing API's for interacting with STM32L476RG MCU
 *		Internal Flash memory. This software is compatible for STM32L476RG
 *		Nucleo board MCU.
 *
 *		@Author: Salman Al Fariz K
 *		@Date:	 30/09/2025
 *****************************************************************************/

#ifndef FLASH_FLASH_H_
#define FLASH_FLASH_H_

/******************************************************************************
 * Global Includes.
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
#define USER_DATA_START_ADDR			(0x08080000)

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
typedef enum _eFlashError_
{
	eFlashSuccess = 0,
	eFlashWriteError,
	eFlashReadError,
	eFlashErrMax
} eFlashError;

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/

/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
eFlashError FlashWriteData(uint8_t* pucData,
		uint16_t uhDataLen, uint32_t uiFlashAddr);

eFlashError FlashReadData(uint8_t* pucData,
		uint16_t uhDataLen, uint32_t uiFlashAddr);

#endif /* FLASH_FLASH_H_ */
