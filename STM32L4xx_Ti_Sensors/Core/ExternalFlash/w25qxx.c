/******************************************************************************
 *		Source File Containing API's for interacting with W25QXX External SPI
 *		Flash memory. This software is compatible for STM32L476RG Nucleo board
 *		MCU.
 *
 *		@Author: Salman Al Fariz K
 *		@Date:	 30/09/2025
 *****************************************************************************/

/******************************************************************************
 * Global Includes.
 *****************************************************************************/
#include <string.h>
#include "w25qxx.h"

/******************************************************************************
 * Macro Definitions.
 *****************************************************************************/
#define W25Q_SECTOR_SIZE					(4096)
#define WRITE_EN							(0x06)
#define WRITE_EN_VOLATILE_STATUS_REG		(0x50)
#define WRITE_DI							(0x04)
#define READ_STATUS_REG1					(0x05)
#define READ_STATUS_REG2					(0x35)
#define READ_STATUS_REG3					(0x15)
#define WRITE_STATUS_REG1					(0x01)
#define WRITE_STATUS_REG2					(0x31)
#define WRITE_STATUS_REG3					(0x11)
#define READ_DATA							(0x03)
#define FAST_READ							(0x0B)
#define FAST_READ_DUAL_OP					(0x3B)
#define FAST_READ_QUAD_OP					(0x6B)
#define FAST_READ_DUAL_IO					(0xBB)
#define FAST_READ_QUAD_IO					(0xEB)
#define SET_BURST_WITH_WRAP					(0x77)
#define PAGE_PGM							(0x02)
#define QUAD_IP_PAGE_PGM					(0x32)
#define SECTOR_ERASE						(0x20)
#define BLOCK_ERASE_32KB					(0x52)
#define BLOCK_ERASE_64KB					(0xD8)
#define CHIP_ERASE							(0xC7)
#define PGM_SUSPEND							(0x75)
#define PGM_RESUME							(0x7A)
#define PWR_DWN								(0xB9)
#define DEVICE_ID							(0xAB)
#define READ_MFID							(0x90)
#define READ_UNIQUE_ID						(0x4B)
#define READ_JEDEC_ID						(0x9F)
#define READ_SFDP_REG						(0x5A)
#define ERASE_SECURITY_REG					(0x44)
#define PGM_SECURITY_REG					(0x42)


/******************************************************************************
 * Enum Definitions.
 *****************************************************************************/
/*
 * @brief: Enum representing slave select controls.
 */
typedef enum _eSlaveSelectControls_
{
	ePullSlaveDown = 0,
	ePullSlaveUp,
	eSlaveControlMax
} eSlaveSelectControls;

/*
 * @brief: Enum representing flash write controls.
 */
typedef enum _eFlashWriteControls_
{
	eFlashWriteEnable = 0,
	eFlashWriteDisable,
	eFlashWriteCtrlMax
} eFlashWriteControls;

/******************************************************************************
 * Structure Definitions.
 *****************************************************************************/

/******************************************************************************
 * Global variable declaration.
 *****************************************************************************/
SPI_HandleTypeDef hspi2;

/******************************************************************************
 * Static Function Declarations.
 *****************************************************************************/
static void MX_SPI2_Init(void);
static void W25QxxSlaveControl(eSlaveSelectControls eControl);
static void W25QxxSectorErase(uint32_t uiSectorAddr);
static void W25QxxFlashWriteControl(eFlashWriteControls eCtrl);
static void W25QxxResetContinousReadMode(void);
static eExtFlashErr W25QxxWaitForReady(void);

/******************************************************************************
 * Function Definitions.
 *****************************************************************************/

/******************************************************************************
 * @brief : Function For Initializing the communication peripheral of W25Qxx
 * 			SPI flash.
 *
 * @fn : FlashWriteData(pucData, uhDataLen, uiFlashAddr);
 *
 * @param[in] : None
 *
 * @param[out] :	None
 *
 * @return : None
 *
 *****************************************************************************/
void InitW25QxxFlash(void)
{
	/* Initialize the SPI peripheral. */
	MX_SPI2_Init();

	//W25QxxResetContinousReadMode();

	return;
}

/******************************************************************************
 * @brief : Function For Writing data to the W25Qxx SPI flash memory on
 * 		    interfacing with STM32L476RG Nucleo board.
 *
 * @fn : W25QxxFlashWriteData(pucData, uhDataLen, uiFlashAddr);
 *
 * @param[in] : pucData		- Reference to the data to be saved.
 * @param[in] : uhDataSize	- Length of the data to be saved.
 * @param[in] : uiSaveAddr	- Address to be saved
 *
 * @param[out] :	None
 *
 * @return : eExtFlashSuccess 		- Success in write operation.
 * @return : eExtFlashWriteError 	- Error in write operation.
 *
 *****************************************************************************/
eExtFlashErr W25QxxFlashWriteData(uint8_t* pucData,
		uint16_t uhDataSize, uint32_t uiSaveAddr)
{
    /* Variable initialization */
    eExtFlashErr eFlashErr = eExtFlashWriteError;
    uint8_t ucTxBuff[260] = {0};
    uint32_t currentAddr = uiSaveAddr;
    uint16_t bytesRemaining = uhDataSize;
    uint8_t *dataPointer = pucData;
    uint32_t sectorStart = 0;
    uint32_t sectorEnd = 0;

    /* Perform validity check. */
    if(NULL != pucData
    && NULL != pucData + uiSaveAddr)
    {
    	eFlashErr = eExtFlashSuccess;

    	sectorStart = currentAddr / W25Q_SECTOR_SIZE;

    	sectorEnd = (currentAddr + bytesRemaining - 1) / W25Q_SECTOR_SIZE;

        /* Erase all affected sectors */
        for(uint32_t sector = sectorStart; sector <= sectorEnd; sector++)
        {
        	uint32_t sectorAddress = sector * W25Q_SECTOR_SIZE;

            W25QxxFlashWriteControl(eFlashWriteEnable);

            /* Erase the sector */
            W25QxxSectorErase(sectorAddress);

            W25QxxWaitForReady();
        }

        /* Write data in chunks of 256 bytes (max page size) */
        while(bytesRemaining > 0)
        {
            uint16_t chunkSize = (bytesRemaining > 256) ? 256 : bytesRemaining;

            /* Write Enable before each page program */
            W25QxxFlashWriteControl(eFlashWriteEnable);

            /* Prepare page program command with address */
            ucTxBuff[0] = PAGE_PGM;
            ucTxBuff[1] = (currentAddr >> 16) & 0xFF;  // Address MSB
            ucTxBuff[2] = (currentAddr >> 8) & 0xFF;   // Address middle byte
            ucTxBuff[3] = currentAddr & 0xFF;          // Address LSB

            /* Copy data to transmit buffer */
            memcpy(ucTxBuff + 4, dataPointer, chunkSize);

            /* Pull down CS and transmit */
            W25QxxSlaveControl(ePullSlaveDown);

            if(HAL_SPI_Transmit(&hspi2, ucTxBuff, chunkSize + 4, 100) != HAL_OK)
            {
                W25QxxSlaveControl(ePullSlaveUp);

                eFlashErr =  eExtFlashWriteError;

                break;
            }

            W25QxxSlaveControl(ePullSlaveUp);

            W25QxxWaitForReady();

            /* Update pointers and counters */
            currentAddr += chunkSize;
            dataPointer += chunkSize;
            bytesRemaining -= chunkSize;
        }

        /* Disable write protection (optional but recommended) */
        W25QxxFlashWriteControl(eFlashWriteDisable);
    }

    return eFlashErr;
}

/******************************************************************************
 * @brief :  Function For Reading data from the W25Qxx SPI flash memory on
 * 		     interfacing with STM32L476RG Nucleo board.
 *
 * @fn : W25QxxFlashReadData(pucData, uhDataSize, uiReadAddr);
 *
 * @param[in] : pucData		- Reference to the data to be stored.
 * @param[in] : uhDataSize	- Length of the data to be read.
 * @param[in] : uiReadAddr	- Address to be read
 *
 * @param[out] : pucData	- Data read from external flash memory.
 *
 * @return : eExtFlashSuccess 		- Success in read operation.
 * @return : eExtFlashReadError 	- Error in read operation.
 *
 *****************************************************************************/
eExtFlashErr W25QxxFlashReadData(uint8_t* pucData, uint16_t uhDataSize,
								uint32_t uiReadAddr)
{
	/* Variable initialization. */
	eExtFlashErr eFlashErr = eExtFlashReadError;
    uint8_t ucTxBuff[5] = {0};
    uint8_t ucCmdSize = 0;

	/* Validity check. */
	if((NULL != pucData)
	&& (NULL != pucData + uhDataSize))
	{
		/* Prepare read command (0x03) and 24-bit address */
	    ucTxBuff[0] = READ_DATA;
	    ucTxBuff[1] = (uiReadAddr >> 16) & 0xFF;
	    ucTxBuff[2] = (uiReadAddr >> 8) & 0xFF;
	    ucTxBuff[3] = uiReadAddr & 0xFF;
	    ucCmdSize = 4;

	    /* Pull down CS to select flash */
	    W25QxxSlaveControl(ePullSlaveDown);

	    /* Transmit read command with address */
	    if(HAL_SPI_Transmit(&hspi2, ucTxBuff, ucCmdSize, 10) == HAL_OK)
	    {
	        /* Receive the data */
	        if(HAL_SPI_Receive(&hspi2, pucData, uhDataSize, 10) == HAL_OK)
	        {
	            eFlashErr = eExtFlashSuccess;
	        }
	    }

	    /* Pull up CS to deselect flash */
	    W25QxxSlaveControl(ePullSlaveUp);
	}

	/* return the error status. */
	return eFlashErr;
}

/******************************************************************************
 * @brief :  Function For Reading data from the W25Qxx SPI flash memory
 * 			JEDEC Id.
 *
 * @fn : GetW25QxxJEDECId();
 *
 * @param[in] : None.
 *
 * @param[out] : None.
 *
 * @return : uint32_t - JEDEC Id.
 *
 *****************************************************************************/
uint32_t GetW25QxxJEDECId(void)
{
	/* Variable Initialization. */
	uint32_t uiJEDECId = 0;
	uint8_t ucTxData = READ_JEDEC_ID;
	uint8_t ucRxBuff[3] = {0};

	/* Pull Down the SPI slave. */
	W25QxxSlaveControl(ePullSlaveDown);

	/* Transmit the JEDEC command. */
	if(HAL_OK == HAL_SPI_Transmit(&hspi2, &ucTxData, 1, 10))
	{
		/* Read 3 bytes of JEDEC ID. */
		if(HAL_OK == HAL_SPI_Receive(&hspi2, ucRxBuff, 3, 10))
		{
			/* Store the JEDEC ID. */
			uiJEDECId = (((uint32_t)ucRxBuff[0] << 16)|
						 ((uint32_t)ucRxBuff[1] <<  8)|
						 ((uint32_t)ucRxBuff[2]));
		}
	}

	/* Pull Up the slave. */
	W25QxxSlaveControl(ePullSlaveUp);

	/* Return the JEDEC ID. */
	return uiJEDECId;
}

/******************************************************************************
 * @brief :  Function For Controlling chip select line of SPI peripheral
 * 			 Interfaced with W25Qxx Flash device.
 *
 * @fn : W25QxxSlaveControl(eControl);
 *
 * @param[in] : ePullSlaveDown - Pull CS line to high.
 * @param[in] : ePullSlaveUp   - Pull CS line to low.
 *
 * @param[out] : None.
 *
 * @return : None.
 *
 *****************************************************************************/
static void W25QxxSlaveControl(eSlaveSelectControls eControl)
{
	/* Validation Check. */
	if(eControl >= ePullSlaveDown
	&& eControl < eSlaveControlMax)
	{
		/* Write the selected option to the GPIO line. */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, eControl);
	}

	return;
}

/******************************************************************************
 * @brief :  Function For Controlling chip select line of SPI peripheral
 * 			 Interfaced with W25Qxx Flash device.
 *
 * @fn : W25QxxSlaveControl(eControl);
 *
 * @param[in] : ePullSlaveDown - Pull CS line to high.
 * @param[in] : ePullSlaveUp   - Pull CS line to low.
 *
 * @param[out] : None.
 *
 * @return : None.
 *
 *****************************************************************************/
static void W25QxxSectorErase(uint32_t uiSectorAddr)
{
	/*Variable Initialization. */
	uint8_t ucTxData[4] = {0};

	ucTxData[0] = SECTOR_ERASE;

	ucTxData[1] = (uiSectorAddr >> 16) & 0xFF;

	ucTxData[2] = (uiSectorAddr >> 8) & 0xFF;

	ucTxData[3] = uiSectorAddr & 0xFF;

	/* Pull down the SPI slave. */
	W25QxxSlaveControl(ePullSlaveDown);

	/* Transmit the sector erase command followed by sector address. */
	HAL_SPI_Transmit(&hspi2, ucTxData, 4, 10);

	/* Pull Up the slave. */
	W25QxxSlaveControl(ePullSlaveUp);

	return;
}

/******************************************************************************
 * @brief :  Function For Controlling chip select line of SPI peripheral
 * 			 Interfaced with W25Qxx Flash device.
 *
 * @fn : W25QxxSlaveControl(eControl);
 *
 * @param[in] : ePullSlaveDown - Pull CS line to high.
 * @param[in] : ePullSlaveUp   - Pull CS line to low.
 *
 * @param[out] : None.
 *
 * @return : None.
 *
 *****************************************************************************/
static void W25QxxFlashWriteControl(eFlashWriteControls eCtrl)
{
	/* Variable Initialization. */
	uint8_t ucTxData = 0;

	/* Validity Check. */
	if(eCtrl >= eFlashWriteEnable
	&& eCtrl < eFlashWriteCtrlMax)
	{
		/* Select the corresponding option. */
		if(eFlashWriteEnable == eCtrl)
		{
			ucTxData = WRITE_EN;
		}

		else if(eFlashWriteDisable == eCtrl)
		{
			ucTxData = WRITE_DI;
		}

		/* Pull down the SPI slave. */
		W25QxxSlaveControl(ePullSlaveDown);

		/* Transmit the write enable\disable command. */
		HAL_SPI_Transmit(&hspi2, &ucTxData, 1, 10);

		/* Pull Up the slave. */
		W25QxxSlaveControl(ePullSlaveUp);
	}

	return;
}

/******************************************************************************
 * @brief :  Function For Controlling chip select line of SPI peripheral
 * 			 Interfaced with W25Qxx Flash device.
 *
 * @fn : W25QxxSlaveControl(eControl);
 *
 * @param[in] : ePullSlaveDown - Pull CS line to high.
 * @param[in] : ePullSlaveUp   - Pull CS line to low.
 *
 * @param[out] : None.
 *
 * @return : None.
 *
 *****************************************************************************/
static eExtFlashErr W25QxxWaitForReady(void)
{
    uint8_t status;
    uint8_t cmd = READ_STATUS_REG1;

    do {
        // Pull CS LOW
        W25QxxSlaveControl(ePullSlaveDown);

        // Send Read Status Register command
        HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);

        // Receive status byte
        HAL_SPI_Receive(&hspi2, &status, 1, 10);

        // Pull CS HIGH
        W25QxxSlaveControl(ePullSlaveUp);

    } while (status & 0x01); // Check if BUSY bit is set

    return eExtFlashSuccess;
}

/******************************************************************************
 * @brief :  Function For Controlling chip select line of SPI peripheral
 * 			 Interfaced with W25Qxx Flash device.
 *
 * @fn : W25QxxSlaveControl(eControl);
 *
 * @param[in] : ePullSlaveDown - Pull CS line to high.
 * @param[in] : ePullSlaveUp   - Pull CS line to low.
 *
 * @param[out] : None.
 *
 * @return : None.
 *
 *****************************************************************************/
static void W25QxxResetContinousReadMode(void)
{
	uint8_t ucCmd = 0xFF;

    // Pull CS LOW
    W25QxxSlaveControl(ePullSlaveDown);

    // Send Read Status Register command
    HAL_SPI_Transmit(&hspi2, &ucCmd, 1, 10);

    // Pull CS HIGH
    W25QxxSlaveControl(ePullSlaveUp);

    return;
}

/******************************************************************************
 * @brief : Static Function For Initializing the communication peripheral SPI
 * 			of STM32L476RG Nucleo that is interfaced with W25Qxx SPI flash.
 *
 * @fn : FlashWriteData(pucData, uhDataLen, uiFlashAddr);
 *
 * @param[in] : None
 *
 * @param[out] :	None
 *
 * @return : None
 *
 *****************************************************************************/

static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}
