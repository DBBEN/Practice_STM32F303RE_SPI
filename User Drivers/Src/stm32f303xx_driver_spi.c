/*
 * stm32f303xx_driver_spi.c
 *
 *  Created on: 17 Jun 2020
 *      Author: Dave
 */
#include "stm32f303xx_driver_spi.h"


/******************************************************
 * 				[ spi_portClk() ]
 *
 * 	@brief		- Enable or Disable SPIx Clock
 *
 * 	[param1]	- SPIx base address
 * 	[param2]	- Enable or Disable
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_portClk(SPI_Typedef *pSPIx, uint8_t enOrDis)
{
	if (enOrDis == ENABLE)
	{
		//Check the SPI number and enable
		if (pSPIx == SPI1) RCC_SPI1_CLKEN();
		else if (pSPIx == SPI2) RCC_SPI2_CLKEN();
		else if (pSPIx == SPI3) RCC_SPI3_CLKEN();
		else if (pSPIx == SPI4) RCC_SPI4_CLKEN();
	}

	else
	{
		//Check the SPI number and disable
		if (pSPIx == SPI1) RCC_SPI1_CLKDIS();
		else if (pSPIx == SPI2) RCC_SPI2_CLKDIS();
		else if (pSPIx == SPI3) RCC_SPI3_CLKDIS();
		else if (pSPIx == SPI4) RCC_SPI4_CLKDIS();
	}
}

/******************************************************
 * 				[ spi_init() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- SPI Handle Structure
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_init(SPI_Handle *pSPIHandle)
{
	spi_portClk(pSPIHandle->pSPIx, ENABLE);

	//temporary register
	uint32_t tempReg = 0;

	//configure device mode
	tempReg |= ( pSPIHandle->SPIConfig.SPI_DeviceMode << 2 );

	//VIDEO #135
	//configure bus configuration
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD ) tempReg &= ~( 1 << 15 ); //Bi-di mode should be cleared
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD ) tempReg |= (1 << 15 ); //Bi-di mode should be set
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SMPLX_RXONLY ){
		//Bi-di mode should be cleared
		tempReg &= ~( 1 << 15 );
		//RXONLY bit must be set
		tempReg |= ( 1 << 10 );
	}

	//configure the SPI serial clock speed (baudrate)
	tempReg |= ( pSPIHandle->SPIConfig.SPI_SclkSpeed << 3 );

	//configure DFF / Data size
	pSPIHandle->pSPIx->CR2 |= ( pSPIHandle->SPIConfig.SPI_DFF << 8 );

	//configure CPOL
	tempReg |= ( pSPIHandle->SPIConfig.SPI_CPOL << 1 );

	//configure CPHA
	tempReg |= ( pSPIHandle->SPIConfig.SPI_CPHA << 0 );

	//configure SSM
	tempReg |= ( pSPIHandle->SPIConfig.SPI_SSM << 9 );

	//Write to CR1 Register
	pSPIHandle->pSPIx->CR1 = tempReg;
}

/******************************************************
 * 				[ spi_deInit() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_deInit(SPI_Typedef *pSPIx)
{
	if (pSPIx == SPI1) RCC_SPI1_RST();
	else if (pSPIx == SPI2) RCC_SPI2_RST();
	else if (pSPIx == SPI3) RCC_SPI3_RST();
	else if (pSPIx == SPI4) RCC_SPI4_RST();
}

/******************************************************
 * 				[ spi_SSIConfig() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_SSIConfig(SPI_Typedef *pSPIx, uint8_t enOrDis)
{
	if(enOrDis == ENABLE) pSPIx->CR1 |= ( 1 << 8 );
	else pSPIx->CR1 &= ~( 1 << 8 );
}

/******************************************************
 * 				[ spi_SSOEConfig() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_SSOEConfig(SPI_Typedef *pSPIx, uint8_t enOrDis)
{
	if(enOrDis == ENABLE) pSPIx->CR2 |= ( 1 << 2 );
	else pSPIx->CR2 &= ~( 1 << 2 );
}

/******************************************************
 * 				[ spi_NSSPConfig() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_NSSPConfig(SPI_Typedef *pSPIx, uint8_t enOrDis)
{
	if(enOrDis == ENABLE) pSPIx->CR2 |= ( 1 << 3 );
	else pSPIx->CR2 &= ~( 1 << 3 );
}

/******************************************************
 * 				[ spi_sendData() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_sendData(SPI_Typedef *pSPIx, uint8_t *pTXbuffer, uint32_t Len)
{

	while(Len > 0)
	{
		//wait until TXE is set (TX buffer empty)
		while(spi_getFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check Data size bit
		//if( (pSPIx->CR2 & (0xF << 8)) )
		if ((((1 << 8) - 1) & (pSPIx->CR2 >> 8)) <= SPI_DFF_8BITS)
		{
			//if 8 bits
			*((uint8_t *)&pSPIx->DR) = *pTXbuffer;
			Len--;
			pTXbuffer++;
		}

		else
		{
			//if 16 bits
			pSPIx->DR = *((uint16_t*)pTXbuffer);
			Len--;
			Len--;
			(uint16_t*)pTXbuffer++;
		}
	}
}

/******************************************************
 * 				[ spi_sendData() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_peripheralControl(SPI_Typedef *pSPIx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE) pSPIx->CR1 |= ( 1 << 6 );
	else pSPIx->CR1 &= ~( 1 << 6 );
}

/******************************************************
 * 				[ spi_receiveData() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_receiveData(SPI_Typedef *pSPIx, uint8_t *pRXbuffer, uint32_t Len)
{

}

/******************************************************
 * 				[ spi_sendData() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
uint8_t spi_getFlagStatus(SPI_Typedef *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) return FLAG_SET;
	return FLAG_RESET;
}

/******************************************************
 * 				[ spi_IRQPriorityConfig() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

/******************************************************
 * 				[ spi_IRQInterruptConfig() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_IRQInterruptConfig(uint8_t IRQNumber,uint8_t enOrDis)
{

}

/******************************************************
 * 				[ spi_IRQhandling() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void spi_IRQhandling(SPI_Handle *pHandle)
{

}
