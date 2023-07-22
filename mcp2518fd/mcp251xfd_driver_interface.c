#include "stm32f1xx_hal.h"
#include "MCP251XFD.h"
#include "CRC16_CMS.h"
#include "ErrorsDef.h"

//=============================================================================
// MCP251XFD_X get millisecond
//=============================================================================
uint32_t GetCurrentms(void)
{
    return HAL_GetTick();
}

//=============================================================================
// MCP251XFD_X compute CRC16-CMS
//=============================================================================
uint16_t ComputeCRC16(const uint8_t* data, size_t size)
{
    return ComputeCRC16CMS(data, size);
}

//*******************************************************************************************************************

#define SPI_NSS_LOW()	( GPIOA->BRR  = 1 << 4 )
#define SPI_NSS_HIGH() 	( GPIOA->BSRR = 1 << 4 )

//=============================================================================
// MCP251XFD SPI driver interface configuration
//=============================================================================
eERRORRESULT MCP251XFD_InterfaceInit(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq)
{
	(void)chipSelect;

	uint16_t brcode = MCP251XFD_DRIVER_SAFE_RESET_SPI_CLK == sckFreq ? 4 : 1;

	CLEAR_BIT  ( SPI1->CR1, SPI_CR1_SPE );
	MODIFY_REG ( SPI1->CR1, 7uL << 3, brcode << 3 );
	SET_BIT    ( SPI1->CR1, SPI_CR1_SPE );

    return ERR_OK;
}

//=============================================================================
// MCP251XFD SPI transfer data
//=============================================================================
eERRORRESULT MCP251XFD_InterfaceTransfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size)
{
	uint8_t tmp[size];

	uint8_t *pTxData = txData ? txData : &tmp[0];
	uint8_t *pRxData = rxData ? rxData : &tmp[0];
	/*
	 * Tricky manipulations with pointers are needed for the next reason.
	 * The HAL_SPI_Transmit Receive() call returns an error if at least one of the pointers
	 * to the transmit/receive buffers passed to it is NULL. At the same time, the MCP251XFD library can pass
	 * one of the pointers equal to NULL, and that pointer is sent to the MCP251XFD_InterfaceTransfer().
	 * To avoid an error in calling HAL_SPI_Transmit Receive(), a null pointer is equated to a pointer to a temporary buffer.
	 * After debugging, it may be necessary to fix this chaos.
	 */

	static const uint32_t Timeout = 10;

	SPI_NSS_LOW();

	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive( pIntDev, pTxData, pRxData, size, Timeout );

	SPI_NSS_HIGH();

	eERRORRESULT err = ERR_OK;

	switch( ret ) {
	case HAL_TIMEOUT:
		err = ERR__TIMEOUT;
		break;

	case HAL_BUSY:
		err = ERR__BUSY;
		break;

	case HAL_ERROR:
		err = ERR__NOT_READY;
		break;

	default:
		break;
	}

    return err;
}
