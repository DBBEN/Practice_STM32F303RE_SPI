/*
 * spi_TXArduinoCom_testing.c
 *
 *  Created on: 20 Jun 2020
 *      Author: Dave
 */

/*
 * Full Duplex
 * 	STM32 -> Master
 * 	Arduino -> Slave
 * DFF = 8bits
 * SSM = 0
 * SCLK speed = DIV_2
 */
#include <string.h>
#include "stm32f303xx.h"

void delay(void)
{
	for(uint32_t i = 0; i <= 300000; i++);
}

void gpio_buttonInit(void)
{
	GPIO_Handle board_BTN;
	board_BTN.pGPIOx = GPIOC;
	board_BTN.GPIOconfig.pinNumber = 13;
	board_BTN.GPIOconfig.pinMode = GPIO_MODE_INPUT;
	board_BTN.GPIOconfig.pinSpeed = GPIO_OP_SPEED_HIGH;
	board_BTN.GPIOconfig.pinPuPd = GPIO_PIN_PUPD_NONE;
	gpio_init(&board_BTN);
}

void gpio_SPI2Init(void)
{
	GPIO_Handle SPIpins;
	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIOconfig.pinMode = GPIO_MODE_AF;
	SPIpins.GPIOconfig.pinAFMode = 5;
	SPIpins.GPIOconfig.pinOPType = GPIO_OP_TYPE_PUPL;
	SPIpins.GPIOconfig.pinPuPd = GPIO_PIN_PUPD_PU;
	SPIpins.GPIOconfig.pinSpeed = GPIO_OP_SPEED_HIGH;

	//SCLK
	SPIpins.GPIOconfig.pinNumber = GPIO_PIN13;
	gpio_init(&SPIpins);

	//MOSI
	SPIpins.GPIOconfig.pinNumber = GPIO_PIN15;
	gpio_init(&SPIpins);

	//MISO
	//SPIpins.GPIOconfig.pinNumber = GPIO_PIN14;
	//gpio_init(&SPIpins);

	//NSS
	SPIpins.GPIOconfig.pinNumber = GPIO_PIN12;
	gpio_init(&SPIpins);
}


void spi2_init(void)
{
	SPI_Handle SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4; //2MHZ SCLK
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; //Hardware Slave Management

	spi_init(&SPI2Handle);
}

int main(void)
{
	char user_data[] = "Hello World";

	gpio_buttonInit();
	gpio_SPI2Init();

	spi2_init();

	//makes NSS signal internally high and avoid MODF error (Master taken over)
	//spi_SSIConfig(SPI2, ENABLE);
	spi_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(gpio_readInputPin(GPIOC, GPIO_PIN13));
		delay();

		//Enable SPI2 Peripheral
		spi_peripheralControl(SPI2, ENABLE);

		uint8_t dataLen = strlen(user_data);
		//send first the length of information
		spi_sendData(SPI2, &dataLen, 1);

		//send data
		spi_sendData(SPI2, (uint8_t*)user_data, strlen(user_data));


		//wait until not busy
		while( spi_getFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//Disable SPI2 Peripheral
		spi_peripheralControl(SPI2, DISABLE);
	}
}


