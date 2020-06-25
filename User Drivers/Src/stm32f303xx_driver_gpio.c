/*
 * stm32f303re_driver_gpio.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Dave
 */
#include "stm32f303xx_driver_gpio.h"

/******************************************************
 * 				[ gpio_portClck() ]
 *
 * 	@brief		- Enable or Disable the GPIO Port Clock
 *
 * 	[param1]	- Base address of the GPIO peripheral
 *
 * 	[param2] 	- Enable or Disable Macro
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_portClk(GPIO_Typedef *pGPIOx, uint8_t enOrDis)
{
	//Check first if enable or disable
	if (enOrDis == ENABLE)
	{
		//Check the GPIO port and enable
		if (pGPIOx == GPIOA) RCC_GPIOA_CLKEN();
		else if (pGPIOx == GPIOB) RCC_GPIOB_CLKEN();
		else if (pGPIOx == GPIOC) RCC_GPIOC_CLKEN();
		else if (pGPIOx == GPIOD) RCC_GPIOD_CLKEN();
		else if (pGPIOx == GPIOE) RCC_GPIOE_CLKEN();
		else if (pGPIOx == GPIOF) RCC_GPIOF_CLKEN();
		else if (pGPIOx == GPIOG) RCC_GPIOG_CLKEN();
		else if (pGPIOx == GPIOH) RCC_GPIOH_CLKEN();
	}

	else
	{
		//Check the GPIO port and disable
		if (pGPIOx == GPIOA) RCC_GPIOA_CLKDIS();
		else if (pGPIOx == GPIOB) RCC_GPIOB_CLKDIS();
		else if (pGPIOx == GPIOC) RCC_GPIOC_CLKDIS();
		else if (pGPIOx == GPIOD) RCC_GPIOD_CLKDIS();
		else if (pGPIOx == GPIOE) RCC_GPIOE_CLKDIS();
		else if (pGPIOx == GPIOF) RCC_GPIOF_CLKDIS();
		else if (pGPIOx == GPIOG) RCC_GPIOG_CLKDIS();
		else if (pGPIOx == GPIOH) RCC_GPIOH_CLKDIS();
	}
}


/******************************************************
 * 				[ gpio_init() ]
 *
 * 	@brief		- Initialize the given GPIO port
 *
 * 	[param1]	- All the GPIO Handle Initializations
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_init(GPIO_Handle *pGPIOHandle)
{
	//Enable clock
	gpio_portClk(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t tempReg = 0; //temporary register
	//Configure GPIO Pin Mode
	//---------------------------------------
	//Non-interrupt Mode
	if( pGPIOHandle->GPIOconfig.pinMode <= GPIO_MODE_ANALOG )	//If GPIO pinMode is non-interrupt mode
	{
		//Configure GPIO pinMode
		tempReg = pGPIOHandle->GPIOconfig.pinMode << ( 2 * pGPIOHandle->GPIOconfig.pinNumber );	//Shift according to pin number by 2 bits
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIOconfig.pinNumber );	//Clear first the bit position before setting
		pGPIOHandle->pGPIOx->MODER |= tempReg;	//setting bit position
		tempReg = 0;

	}
	//---------------------------------------
	//Interrupt Mode
	else
	{
		//Configuring Interrupt trigger selection modes
		if(pGPIOHandle->GPIOconfig.pinMode == GPIO_IT_FALLING)
		{
			//configure FTSR (Falling Trigger Selection Register)
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIOconfig.pinNumber ); //disable Rising Trigger
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIOconfig.pinNumber ); //enable Falling Trigger
		}

		else if(pGPIOHandle->GPIOconfig.pinMode == GPIO_IT_RISING)
		{
			//configure RTSR (Rising Trigger Selection Register)
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIOconfig.pinNumber ); //disable Falling Trigger
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIOconfig.pinNumber ); //enable Rising Trigger
		}

		else if(pGPIOHandle->GPIOconfig.pinMode == GPIO_IT_RISING_FALLING)
		{
			//configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIOconfig.pinNumber ); //enable Falling Trigger
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIOconfig.pinNumber ); //enable Rising Trigger
		}

		//configure which GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIOconfig.pinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIOconfig.pinNumber % 4;
		uint8_t SYSCFGportCode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
		RCC_SYSCFG_EN(); //enable clock for SYSCFG
		SYSCFG->EXTICR[temp1] |= SYSCFGportCode << ( 4 * temp2 );

		//enable EXTI interrupt delivery with IMR (Interrupt Mask Register)
		EXTI->IMR |= 1 << pGPIOHandle->GPIOconfig.pinNumber;
	}

	//Configure GPIO pinSpeed
	tempReg = pGPIOHandle->GPIOconfig.pinSpeed << ( 2 * pGPIOHandle->GPIOconfig.pinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIOconfig.pinNumber );	//Clear first the bit position before setting
	pGPIOHandle->pGPIOx->OSPEEDR |= tempReg;
	tempReg = 0;

	//Configure Pull-up/Pull-down settings
	tempReg = pGPIOHandle->GPIOconfig.pinPuPd << ( 2 * pGPIOHandle->GPIOconfig.pinNumber );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIOconfig.pinNumber );	//Clear first the bit position before setting
	pGPIOHandle->pGPIOx->PUPDR |= tempReg;
	tempReg = 0;

	//Configure Output type
	tempReg = pGPIOHandle->GPIOconfig.pinOPType <<  pGPIOHandle->GPIOconfig.pinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIOconfig.pinNumber );	//Clear first the bit position before setting
	pGPIOHandle->pGPIOx->OTYPER |= tempReg;
	tempReg = 0;

	//If AF_MODE, configure Alternate Functionality
	if (pGPIOHandle->GPIOconfig.pinMode == GPIO_MODE_AF)
	{
		//Configure Alternate Function Registers
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIOconfig.pinNumber / 8;
		temp2 = pGPIOHandle->GPIOconfig.pinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << ( 4 * temp2 ) );	//Clear first the bit positions before setting
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIOconfig.pinAFMode << ( 4 * temp2 ) );
	}


}


/******************************************************
 * 				[ gpio_deInit() ]
 *
 * 	@brief		- Reset all the registers of the given GPIO port
 *
 * 	[param1]	- GPIO Port Base Address
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_deInit(GPIO_Typedef *pGPIOx)
{
	if (pGPIOx == GPIOA) RCC_GPIOA_RST();
	else if (pGPIOx == GPIOB) RCC_GPIOB_RST();
	else if (pGPIOx == GPIOC) RCC_GPIOC_RST();
	else if (pGPIOx == GPIOD) RCC_GPIOD_RST();
	else if (pGPIOx == GPIOE) RCC_GPIOE_RST();
	else if (pGPIOx == GPIOF) RCC_GPIOF_RST();
	else if (pGPIOx == GPIOG) RCC_GPIOG_RST();
	else if (pGPIOx == GPIOH) RCC_GPIOH_RST();
}

/******************************************************
 * 				[ gpio_readInputPin() ]
 *
 * 	@brief		- Read data of a pin from GPIOx
 *
 * 	[param1]	- GPIO Port Base Address
 * 	[param2]	- Pin Number
 	 	 	 	 	 	 	 	 	 	 	 */
uint8_t gpio_readInputPin(GPIO_Typedef *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> pinNumber) & 0x00000001;	//Shift the bit according to the bit position and mask
	return value;
}

/******************************************************
 * 				[ gpio_readInputPort() ]
 *
 * 	@brief		- Read data from entire GPIOx
 *
 * 	[param1]	- GPIO Port Base Address
 	 	 	 	 	 	 	 	 	 	 	 */
uint16_t gpio_readInputPort(GPIO_Typedef *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

/******************************************************
 * 				[ gpio_writeOutputPin() ]
 *
 * 	@brief		- Write data to a pin from GPIOx
 *
 * 	[param1]	- GPIO Port Base Address
 * 	[param2]	- Pin Number
 * 	[param3]	- Value to write (HIGH or LOW)
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_writeOutputPin(GPIO_Typedef *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == HIGH) pGPIOx->ODR |= ( 1 << pinNumber );	//Write 1 to digital output pin
	else	pGPIOx->ODR &= ~( 1 << pinNumber );				//Write 0 to digital output pin
}

/******************************************************
 * 				[ gpio_digitalWrite() ]
 *
 * 	@brief		- Write data to a pin from GPIOx by only mentioning the GPIO variable name (like Arduino digitalWrite())
 *
 * 	[param1]	- GPIO Variable name
 * 	[param2]	- Value to write (HIGH or LOW)
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_digitalWrite(GPIO_Handle newGPIO, uint8_t value)
{
	GPIO_Handle *pnewGPIO = &newGPIO;
	if(value == HIGH) pnewGPIO->pGPIOx->ODR |= ( 1 << pnewGPIO->GPIOconfig.pinNumber );	//Write 1 to digital output pin
	else pnewGPIO->pGPIOx->ODR &= ~( 1 << pnewGPIO->GPIOconfig.pinNumber );			//Write 0 to digital output pin

}

/******************************************************
 * 				[ gpio_digitalRead() ]
 *
 * 	@brief		- Read data to a pin from GPIOx by only mentioning the GPIO variable name (like Arduino digitalRead())
 *
 * 	[param1]	- GPIO Variable name
 	 	 	 	 	 	 	 	 	 	 	 */
uint8_t gpio_digitalRead(GPIO_Handle newGPIO)
{
	uint8_t value;
	GPIO_Handle *pnewGPIO = &newGPIO;
	value = (uint8_t)(pnewGPIO->pGPIOx->IDR >> pnewGPIO->GPIOconfig.pinNumber) & 0x00000001;	//Shift the bit according to the bit position and mask
	return value;
}

/******************************************************
 * 				[ gpio_writeOutputPort() ]
 *
 * 	@brief		- Write data to entire GPIOx
 *
 * 	[param1]	- GPIO Port Base Address
 * 	[param2]	- 16-bit Value to write
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_writeOutputPort(GPIO_Typedef *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/******************************************************
 * 				[ gpio_toggleOutputPin() ]
 *
 * 	@brief		- Toggle data to a pin from GPIOx
 *
 * 	[param1]	- GPIO Port Base Address
 * 	[param2]	- Pin Number
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_toggleOutputPin(GPIO_Typedef *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= ( 1 << pinNumber );
}




//			INTERRUPT HANDLING FUNCTIONS




void gpio_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	//https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/learn/lecture/14277518#questions
	uint8_t shiftAmount = ( 8 * IPRx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*( CPU_MVIC_IPR_BASE + IPRx ) |= ( IRQPriority << shiftAmount  );
}


/******************************************************
 * 				[ gpio_IRQconfig() ]
 *
 * 	@brief		- Cortex-M4 Processor specific function
 *
 * 	[param1]	- IRQ Number
 * 	[param2]	- IRQ Priority Number
 * 	[param3]	- Enable or Disable
 	 	 	 	 	 	 	 	 	 	 	 */
void gpio_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDis)
{
	if(enOrDis == ENABLE)
	{
		if(IRQNumber <= 31) *CPU_NVIC_ISER0 |= ( 1 << IRQNumber ); //IRQNumber =  0-31, program ISER0 register to enable IRQ number
		else if(IRQNumber > 31 && IRQNumber < 64) *CPU_NVIC_ISER1 |= ( 1 << (IRQNumber % 32) ); //IRQNumber =  32-63, program ISER1 register to enable IRQ number
		else if(IRQNumber >= 64 && IRQNumber < 96) *CPU_NVIC_ISER2 |= ( 1 << (IRQNumber % 64) ); //IRQNumber =  64-95, program ISER2 register to enable IRQ number
	}

	else
	{
		if(IRQNumber <= 31) *CPU_NVIC_ICER0 |= ( 1 << IRQNumber ); //IRQNumber =  0-31, program ICER0 register
		else if(IRQNumber > 31 && IRQNumber < 64) *CPU_NVIC_ICER1 |= ( 1 << (IRQNumber % 32) ); //IRQNumber =  32-63, program ICER1 register to disable IRQ number
		else if(IRQNumber >= 64 && IRQNumber < 96) *CPU_NVIC_ICER2 |= ( 1 << (IRQNumber % 64) ); //IRQNumber =  64-95, program ICER2 register to disable IRQ number
	}
}

void gpio_IRQhandling(uint8_t pinNumber)
{
	//clear EXTI Pending Register corresponding to the pin
	if(EXTI->PR & ( 1 << pinNumber ) ) EXTI->PR |= ( 1 << pinNumber );
}

