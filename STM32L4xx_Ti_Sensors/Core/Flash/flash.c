/******************************************************************************
 *		Header File Containing API's for interacting with STM32L476RG MCU
 *		Internal Flash memory. This software is compatible for STM32L476RG
 *		Nucleo board MCU.
 *
 *		@Author: Salman Al Fariz K
 *		@Date:	 30/09/2025
 *****************************************************************************/

/******************************************************************************
 * Global Includes.
 *****************************************************************************/
#include <string.h>
#include "flash.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/

/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/

/******************************************************************************
 * Static Function Declarations.
 *****************************************************************************/
static void InitFlashEraseStruct(FLASH_EraseInitTypeDef* ptEraseInitStruct);

/******************************************************************************
 * Function Definitions.
 *****************************************************************************/

/******************************************************************************
 * @brief : Function For Writing data to the internal flash memory of
 * 		=   STM32L476RG MCU.
 *
 * @fn : FlashWriteData(pucData, uhDataLen, uiFlashAddr);
 *
 * @param[in] : pucData		- Reference to the data to be saved.
 * @param[in] : uhDataLen	- Length of the data to be saved.
 * @param[in] : uiFlashAddr	- Address to be saved
 *
 * @param[out] :	None
 *
 * @return : eFlashSuccess 		- Success in write operation.
 * @return : eFlashWriteError 	- Error in write operation.
 *
 *****************************************************************************/
eFlashError FlashWriteData(uint8_t* pucData,
		uint16_t uhDataLen, uint32_t uiFlashAddr)
{
	/* Variable Initialization. */
	eFlashError eFlashErr = eFlashWriteError;
	uint32_t uiPageError = 0;
	uint64_t ulData = 0;
	FLASH_EraseInitTypeDef tEraseInitStruct = {0};

	/* Validity Check. */
	if((NULL != pucData)
	&& (NULL != pucData + uhDataLen))
	{
		/* Unlock The Flash. */
		if(HAL_OK == HAL_FLASH_Unlock())
		{
			/* Initialize the flash erase structure. */
			InitFlashEraseStruct(&tEraseInitStruct);

			/* Erase the Flash before programming. */
			if(HAL_OK == HAL_FLASHEx_Erase(&tEraseInitStruct,
					&uiPageError))
			{
				/* Update the error status as success. */
				eFlashErr = eFlashSuccess;

				/* Write the user data into the flash memory. */
				for(uint32_t uiIdx = 0; uiIdx < uhDataLen; uiIdx += 8)
				{
					/* Calculate for length to be programmed. */
					if(uhDataLen - uiIdx  < 8)
					{
						memcpy(&ulData, pucData + uiIdx, uhDataLen - uiIdx);
					}
					else
					{
						memcpy(&ulData, pucData + uiIdx, 8);
					}

					/* Program the user data. */
					if( HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							uiFlashAddr + uiIdx, ulData))
					{
						/* Error in case failure. */
						eFlashErr = eFlashWriteError;

						/* Exit the loop. */
						break;
					}
				}
			}
		}

		HAL_FLASH_Lock();
	}

	/* Return the error status. */
	return eFlashErr;
}

/******************************************************************************
 * @brief : Function For reading data from the internal flash memory of
 * 		=   STM32L476RG MCU.
 *
 * @fn : FlashReadData(pucData, uhDataLen, uiFlashAddr);
 *
 * @param[in] : pucData		- Reference to the data to be stored after read.
 * @param[in] : uhDataLen	- Length of the data to be read from flash.
 * @param[in] : uiFlashAddr	- Address to be read from flash.
 *
 * @param[out] : pucData 		- Data read from flash memory.
 *
 * @return : eFlashSuccess 		- Success in read operation.
 * @return : eFlashReadError 	- Error in read operation.
 *
 *****************************************************************************/
eFlashError FlashReadData(uint8_t* pucData,
		uint16_t uhDataLen, uint32_t uiFlashAddr)
{
	/* Variable Initialization. */
	eFlashError eFlashErr = eFlashReadError;

	/* Validity check. */
	if((NULL != pucData)
	&& (NULL != pucData + uhDataLen))
	{
		for(uint32_t uiIdx = 0; uiIdx < uhDataLen; uiIdx += 8)
		{
			if(uhDataLen - uiIdx < 8)
			{
				memcpy(pucData + uiIdx, (uint64_t*)(uiFlashAddr + uiIdx),
						uhDataLen - uiIdx);
			}
			else
			{
				memcpy(pucData + uiIdx, (uint64_t*)(uiFlashAddr + uiIdx),8);
			}
		}

		/* Update the error status. */
		eFlashErr = eFlashSuccess;
	}

	/* return the error status. */
	return eFlashErr;
}

/******************************************************************************
 * @brief : Function For Initializing the structure required for flash erase
 * 			API.
 *
 * @fn : InitFlashEraseStruct(&tEraseInitStruct);
 *
 * @param[in] : ptEraseInitStruct - Reference initialization structure.
 *
 * @param[out] : None.
 *
 * @return : None
 *
 *****************************************************************************/
static void InitFlashEraseStruct(FLASH_EraseInitTypeDef* ptEraseInitStruct)
{
	/* Validity Check. */
	if(NULL != ptEraseInitStruct)
	{
		/* Assign bank as bank 1. */
		ptEraseInitStruct->Banks = FLASH_BANK_2;

		/* Take 1 Page. */
		ptEraseInitStruct->NbPages = 1;

		/* Last Page of Bank 1*/
		ptEraseInitStruct->Page = 0;

		/* Select the page programming option. */
		ptEraseInitStruct->TypeErase = FLASH_TYPEERASE_PAGES;
	}

	return;
}
