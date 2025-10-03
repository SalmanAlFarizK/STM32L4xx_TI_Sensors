/******************************************************************************
 *		Header File Containing API's for interacting with W25QXX External SPI
 *		Flash memory. This software is compatible for STM32L476RG Nucleo board
 *		MCU.
 *
 *		@Author: Salman Al Fariz K
 *		@Date:	 30/09/2025
 *****************************************************************************/

#ifndef EXTERNALFLASH_W25QXX_H_
#define EXTERNALFLASH_W25QXX_H_

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
#define USER_ADDR_1				(0x000000)		 /*< Sector 0 starts address */
#define USER_ADDR_2				(0x001000)		 /*< Sector 1 starts address */
#define USER_ADDR_3				(0x002000)		 /*< Sector 2 starts address */

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
typedef enum _eExtFlashErr_
{
	eExtFlashSuccess = 0,
	eExtFlashWriteError,
	eExtFlashReadError,
	eExtFlashErrorMax
} eExtFlashErr;

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/

/******************************************************************************
 * Function Declarations.
 *****************************************************************************/
void InitW25QxxFlash(void);
eExtFlashErr W25QxxFlashWriteData(uint8_t* pucData, uint16_t uhDataSize,
									uint32_t uiSaveAddr);
eExtFlashErr W25QxxFlashReadData(uint8_t* pucData, uint16_t uhDataSize,
									uint32_t uiReadAddr);
uint32_t GetW25QxxJEDECId(void);

#endif /* EXTERNALFLASH_W25QXX_H_ */
