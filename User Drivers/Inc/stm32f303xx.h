/*
 * stm32f303re.h
 *
 *  Created on: Jun 11, 2020
 *      Author: Dave
 *
 *      Self-written user device specific header file for the
 *      STM32F303RE
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_

#include <stdint.h>

#define __vo					volatile

/********************************** [	PROCESSOR SPECIFIC DETAILS	] **********************************
 *
 * ARM Cortex-Mx Processor NVIC ISERx Register Addresses
 */

//	Interrupt Set Registers
#define CPU_NVIC_ISER0					( (__vo uint32_t*) 0xE000E100 )
#define CPU_NVIC_ISER1					( (__vo uint32_t*) 0xE000E104 )
#define CPU_NVIC_ISER2					( (__vo uint32_t*) 0xE000E108 )
#define CPU_NVIC_ISER3					( (__vo uint32_t*) 0xE000E10C )

//	Interrupt Clear Registers
#define CPU_NVIC_ICER0					( (__vo uint32_t*) 0XE000E180 )
#define CPU_NVIC_ICER1					( (__vo uint32_t*) 0XE000E184 )
#define CPU_NVIC_ICER2					( (__vo uint32_t*) 0XE000E188 )
#define CPU_NVIC_ICER3					( (__vo uint32_t*) 0XE000E18C )

//	Interrupt Priority Registers
#define CPU_MVIC_IPR_BASE				( (__vo uint32_t*) 0xE000E400 )

//******************************************************************************************************
#define NO_PR_BITS_IMPLEMENTED		4
//	IRQ (Interrupt Request) Number Position of STM32F303xx MCU
#define IRQn_EXTI0					6
#define IRQn_EXTI1					7
#define IRQn_EXTI2					8
#define IRQn_EXTI3					9
#define IRQn_EXTI4					10
#define IRQn_EXTI9_5				23
#define IRQn_EXTI15_10				40

// IRQ Priorities
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


//	EXTI Peripheral Registers Type Definition Structure
typedef struct
{
	__vo uint32_t IMR;		//Interrupt Mask register 1
	__vo uint32_t EMR;		//Event Mask register 1
	__vo uint32_t RTSR;		//Rising Trigger Selection register 1
	__vo uint32_t FTSR;		//Falling Trigger Selection register 1
	__vo uint32_t SWIER;	//Software Interrupt Event register 1
	__vo uint32_t PR;		//Pending register 1
/*	__vo uint32_t IMR2;		//Interrupt Mask register 2
	__vo uint32_t EMR2;		//Event Mask register 2
	__vo uint32_t RTSR2;	//Rising Trigger Selection register 2
	__vo uint32_t FTSR2;	//Falling Trigger Selection register 2
	__vo uint32_t SWIER2;	//Software Interrupt Event register 2
	__vo uint32_t PR2;		//Pending register 2*/
}EXTI_Typedef;

//	GPIO Peripheral Registers Type Definition Structure
typedef struct
{
	__vo uint32_t MODER;	//Mode register
	__vo uint32_t OTYPER;	//Output Type register
	__vo uint32_t OSPEEDR;	//Output Speed register
	__vo uint32_t PUPDR;	//Pull-up/Pull-down register
	__vo uint32_t IDR;		//Input Data register
	__vo uint32_t ODR;		//Output Data register
	__vo uint32_t BSRR;		//Bit Set/Reset register
	__vo uint32_t LCKR;		//Configuration Lock register
	__vo uint32_t AFR[2];	//Alternate Function (Low[0] and High[1]) register
}GPIO_Typedef;

//	RCC Peripheral Registers Type Definition Structure
typedef struct
{
	__vo uint32_t CR;		//Clock Control register
	__vo uint32_t CFGR;		//Configuration register
	__vo uint32_t CIR;		//Clock Interrupt register
	__vo uint32_t APB2RSTR;	//APB2 Peripheral Reset register
	__vo uint32_t APB1RSTR;	//APB1 Peripheral Reset register
	__vo uint32_t AHBENR;	//AHB Enable register
	__vo uint32_t APB2ENR;	//APB2 Enable register
	__vo uint32_t APB1ENR;	//APB1 Enable register
	__vo uint32_t BDCR;		//RTC Domain register
	__vo uint32_t CSR;		//Control/Status register
	__vo uint32_t AHBRSTR;	//AHB Peripheral Reset register
	__vo uint32_t CFGR2;	//Configuration 2 register
	__vo uint32_t CFGR3;	//Configuration 3 register
}RCC_Typedef;

//	SPI Peripheral Registers Type Definition Structure
typedef struct
{
	__vo uint32_t CR1;			//SPIx Control register 1
	__vo uint32_t CR2;			//SPIx Control register 2
	__vo uint32_t SR;			//SPIx Status register
	__vo uint32_t DR;			//SPIx Data register
	__vo uint32_t CRCPR;		//SPIx CRC Polynomial register
	__vo uint32_t RXCPR;		//SPI RX CRC register
	__vo uint32_t TXCRCR;		//SPI TX CRC register
	__vo uint32_t I2SCFGR;		//SPIx_I2S Configuration register
	__vo uint32_t I2SPR;		//SPIx_I2s Prescaler register
}SPI_Typedef;

//	SYSCFG Peripheral Registers Type Definition Structure
typedef struct
{
	__vo uint32_t CFGR1;		//SYSCFG Configuration register 1
	__vo uint32_t RCR;			//SYSCFG SRAM Configuration register
	__vo uint32_t EXTICR[4];	//SYSCFG EXTI Configuration registers
	__vo uint32_t CFGR2;		//SYSCFG Configuration register 2
	__vo uint32_t CFGR3;		//SYSCFG Configuration register 3
	__vo uint32_t CFGR4;		//SYSCFG Configuration register 4
}SYSCFG_Typedef;


#define FLASH_BASE				( 0x08000000U ) //Flash memory base address
#define SRAM_BASE				( 0x20000000U ) //SRAM base address
#define ROM_BASE				( 0x1FFFD800U ) //System Memory (ROM) base address

#define BUSPERIPH_BASE			( 0x40000000U ) //Peripheral bus base addresses
#define APB1PERIPH_BASE		 	BUSPERIPH_BASE  //APB1 Peripheral base address
#define APB2PERIPH_BASE			( 0x40010000U ) //APB2 Peripheral base address
#define AHB1PERIPH_BASE			( 0x40020000U ) //AHB1 Peripheral base address
#define AHB2PERIPH_BASE			( 0x48000000U ) //AHB2 Peripheral base address
#define AHB3PERIPH_BASE			( 0x50000000U ) //AHB3 Peripheral base address
#define AHB4PERIPH_BASE			( 0x60000000U ) //AHB4 Peripheral base address

// APB1 Peripherals Base Addresses
#define TIM2_BASE				( APB1PERIPH_BASE + 0x0000 ) //Timer 2 Peripheral base address
#define TIM3_BASE				( APB1PERIPH_BASE + 0x0400 ) //Timer 3 Peripheral base address
#define TIM4_BASE				( APB1PERIPH_BASE + 0x0800 ) //Timer 4 Peripheral base address
#define TIM6_BASE				( APB1PERIPH_BASE + 0x1000 ) //Timer 6 Peripheral base address
#define TIM7_BASE				( APB1PERIPH_BASE + 0x1400 ) //Timer 7 Peripheral base address
#define SPI2_BASE				( APB1PERIPH_BASE + 0x3800 ) //SPI 2 Peripheral base address
#define SPI3_BASE				( APB1PERIPH_BASE + 0x3C00 ) //SPI 3 Peripheral base address
#define USART2_BASE				( APB1PERIPH_BASE + 0x3800 ) //USART 2 Peripheral base address
#define USART3_BASE				( APB1PERIPH_BASE + 0x4800 ) //USART 3 Peripheral base address
#define UART4_BASE				( APB1PERIPH_BASE + 0x4C00 ) //UART 4 Peripheral base address
#define UART5_BASE				( APB1PERIPH_BASE + 0x5000 ) //UART 5 Peripheral base address
#define I2C1_BASE				( APB1PERIPH_BASE + 0x5400 ) //I2C 1 Peripheral base address
#define I2C2_BASE				( APB1PERIPH_BASE + 0x5800 ) //I2C 2 Peripheral base address

// APB2 Peripherals Base Addresses
#define SYSCFG_BASE				( APB2PERIPH_BASE + 0x0000 ) //SYSCFG Peripheral base address
#define EXTI_BASE				( APB2PERIPH_BASE + 0x0400 ) //EXTI Peripheral base address
#define TIM1_BASE				( APB2PERIPH_BASE + 0x2C00 ) //Timer 1 Peripheral base address
#define SPI1_BASE				( APB2PERIPH_BASE + 0x3000 ) //SPI 1 Peripheral base address
#define TIM8_BASE				( APB2PERIPH_BASE + 0x3400 ) //Timer 8 Peripheral base address
#define USART1_BASE				( APB2PERIPH_BASE + 0x3800 ) //USART 1 Peripheral base address
#define SPI4_BASE				( APB2PERIPH_BASE + 0x3C00 ) //SPI 4 Peripheral base address
#define TIM15_BASE				( APB2PERIPH_BASE + 0x4000 ) //Timer 15 Peripheral base address
#define TIM16_BASE				( APB2PERIPH_BASE + 0x4400 ) //Timer 16 Peripheral base address
#define TIM17_BASE				( APB2PERIPH_BASE + 0x4800 ) //Timer 17 Peripheral base address
#define TIM20_BASE				( APB2PERIPH_BASE + 0x5000 ) //Timer 20 Peripheral base address


// AHB1 Peripherals Base Addresses
#define RCC_BASE				( AHB1PERIPH_BASE + 0x1000 ) //RCC Peripheral base address

// AHB2 Peripherals Base Addresses
#define GPIOA_BASE				( AHB2PERIPH_BASE + 0x0000 ) //GPIOA base address
#define GPIOB_BASE				( AHB2PERIPH_BASE + 0x0400 ) //GPIOB base address
#define GPIOC_BASE				( AHB2PERIPH_BASE + 0x0800 ) //GPIOC base address
#define GPIOD_BASE				( AHB2PERIPH_BASE + 0x0C00 ) //GPIOD base address
#define GPIOE_BASE				( AHB2PERIPH_BASE + 0x1000 ) //GPIOE base address
#define GPIOF_BASE				( AHB2PERIPH_BASE + 0x1400 ) //GPIOF base address
#define GPIOG_BASE				( AHB2PERIPH_BASE + 0x1800 ) //GPIOG base address
#define GPIOH_BASE				( AHB2PERIPH_BASE + 0x1C00 ) //GPIOH base address

// AHB3 Peripherals Base Addresses


// AHB1 Peripherals Base Addresses



//Peripheral Registers Definitions
#define EXTI					( (EXTI_Typedef *) EXTI_BASE )
#define GPIOA					( (GPIO_Typedef *) GPIOA_BASE )
#define GPIOB					( (GPIO_Typedef *) GPIOB_BASE )
#define GPIOC					( (GPIO_Typedef *) GPIOC_BASE )
#define GPIOD					( (GPIO_Typedef *) GPIOD_BASE )
#define GPIOE					( (GPIO_Typedef *) GPIOE_BASE )
#define GPIOF					( (GPIO_Typedef *) GPIOF_BASE )
#define GPIOG					( (GPIO_Typedef *) GPIOG_BASE )
#define GPIOH					( (GPIO_Typedef *) GPIOH_BASE )
#define RCC						( (RCC_Typedef *) RCC_BASE )
#define SPI1					( (SPI_Typedef *) SPI1_BASE )
#define SPI2					( (SPI_Typedef *) SPI2_BASE )
#define SPI3					( (SPI_Typedef *) SPI3_BASE )
#define SPI4					( (SPI_Typedef *) SPI4_BASE )
#define SYSCFG					( (SYSCFG_Typedef *) SYSCFG_BASE )

/**********************************************************************
 * 					Bit Position Definitions
 **********************************************************************/
#define SPI_CR1_CPHA_BITPOS			0


//Clock Enable Macros ----------------------------------------
#define RCC_GPIOA_CLKEN()			RCC->AHBENR |= ( 1 << 17 )
#define RCC_GPIOB_CLKEN()			RCC->AHBENR |= ( 1 << 18 )
#define RCC_GPIOC_CLKEN()			RCC->AHBENR |= ( 1 << 19 )
#define RCC_GPIOD_CLKEN()			RCC->AHBENR |= ( 1 << 20 )
#define RCC_GPIOE_CLKEN()			RCC->AHBENR |= ( 1 << 21 )
#define RCC_GPIOF_CLKEN()			RCC->AHBENR |= ( 1 << 22 )
#define RCC_GPIOG_CLKEN()			RCC->AHBENR |= ( 1 << 23 )
#define RCC_GPIOH_CLKEN()			RCC->AHBENR |= ( 1 << 16 )

#define RCC_SPI1_CLKEN()			RCC->APB2ENR |= ( 1 << 12 )
#define RCC_SPI2_CLKEN()			RCC->APB1ENR |= ( 1 << 14 )
#define RCC_SPI3_CLKEN()			RCC->APB1ENR |= ( 1 << 15 )
#define RCC_SPI4_CLKEN()			RCC->APB2ENR |= ( 1 << 15 )

#define RCC_USART1_CLKEN()			RCC->APB2ENR |= ( 1 << 14 )
#define RCC_USART2_CLKEN()			RCC->APB1ENR |= ( 1 << 17 )
#define RCC_USART3_CLKEN()			RCC->APB1ENR |= ( 1 << 18 )
#define RCC_UART4_CLKEN()			RCC->APB1ENR |= ( 1 << 19 )
#define RCC_UART5_CLKEN()			RCC->APB1ENR |= ( 1 << 20 )

#define RCC_I2C1_CLKEN()			RCC->APB1ENR |= ( 1 << 21 )
#define RCC_I2C2_CLKEN()			RCC->APB1ENR |= ( 1 << 22 )

#define RCC_SYSCFG_EN()				RCC->APB2ENR |= ( 1 << 0 )

//Clock Disable Macros ----------------------------------------
#define RCC_GPIOA_CLKDIS()			RCC->AHBENR &= ~( 1 << 17 )
#define RCC_GPIOB_CLKDIS()			RCC->AHBENR &= ~( 1 << 18 )
#define RCC_GPIOC_CLKDIS()			RCC->AHBENR &= ~( 1 << 19 )
#define RCC_GPIOD_CLKDIS()			RCC->AHBENR &= ~( 1 << 20 )
#define RCC_GPIOE_CLKDIS()			RCC->AHBENR &= ~( 1 << 21 )
#define RCC_GPIOF_CLKDIS()			RCC->AHBENR &= ~( 1 << 22 )
#define RCC_GPIOG_CLKDIS()			RCC->AHBENR &= ~( 1 << 23 )
#define RCC_GPIOH_CLKDIS()			RCC->AHBENR &= ~( 1 << 16 )

#define RCC_SPI1_CLKDIS()			RCC->APB2ENR &= ~( 1 << 12 )
#define RCC_SPI2_CLKDIS()			RCC->APB1ENR &= ~( 1 << 14 )
#define RCC_SPI3_CLKDIS()			RCC->APB1ENR &= ~( 1 << 15 )
#define RCC_SPI4_CLKDIS()			RCC->APB2ENR &= ~( 1 << 15 )

#define RCC_USART1_CLKDIS()			RCC->APB2ENR &= ~( 1 << 14 )
#define RCC_USART2_CLKDIS()			RCC->APB1ENR &= ~( 1 << 17 )
#define RCC_USART3_CLKDIS()			RCC->APB1ENR &= ~( 1 << 18 )
#define RCC_UART4_CLKDIS()			RCC->APB1ENR &= ~( 1 << 19 )
#define RCC_UART5_CLKDIS()			RCC->APB1ENR &= ~( 1 << 20 )

#define RCC_I2C3_CLKEN()			RCC->APB1ENR |= ( 1 << 30 )
#define RCC_I2C1_CLKDIS()			RCC->APB1ENR &= ~( 1 << 21 )
#define RCC_I2C2_CLKDIS()			RCC->APB1ENR &= ~( 1 << 22 )
#define RCC_I2C3_CLKDIS()			RCC->APB1ENR &= ~( 1 << 30 )

#define RCC_SYSCFG_DIS()			RCC->APB2ENR &= ~( 1 << 0 )

//Reset GPIOx Peripherals Macros ----------------------------------------
#define RCC_GPIOA_RST()				do { (RCC->AHBRSTR |= ( 1 << 17 ));	(RCC->AHBRSTR &= ~( 1 << 17 )); } while(0)	//Set bit and reset GPIOA Reset bit
#define RCC_GPIOB_RST()				do { (RCC->AHBRSTR |= ( 1 << 18 ));	(RCC->AHBRSTR &= ~( 1 << 18 )); } while(0)	//Set bit and reset GPIOB Reset bit
#define RCC_GPIOC_RST()				do { (RCC->AHBRSTR |= ( 1 << 19 ));	(RCC->AHBRSTR &= ~( 1 << 19 )); } while(0)	//Set bit and reset GPIOC Reset bit
#define RCC_GPIOD_RST()				do { (RCC->AHBRSTR |= ( 1 << 20 ));	(RCC->AHBRSTR &= ~( 1 << 20 )); } while(0)	//Set bit and reset GPIOD Reset bit
#define RCC_GPIOE_RST()				do { (RCC->AHBRSTR |= ( 1 << 21 ));	(RCC->AHBRSTR &= ~( 1 << 21 )); } while(0)	//Set bit and reset GPIOE Reset bit
#define RCC_GPIOF_RST()				do { (RCC->AHBRSTR |= ( 1 << 22 ));	(RCC->AHBRSTR &= ~( 1 << 22 )); } while(0)	//Set bit and reset GPIOF Reset bit
#define RCC_GPIOG_RST()				do { (RCC->AHBRSTR |= ( 1 << 23 ));	(RCC->AHBRSTR &= ~( 1 << 23 )); } while(0)	//Set bit and reset GPIOG Reset bit
#define RCC_GPIOH_RST()				do { (RCC->AHBRSTR |= ( 1 << 16 ));	(RCC->AHBRSTR &= ~( 1 << 16 )); } while(0)	//Set bit and reset GPIOH Reset bit

#define RCC_SPI1_RST()				do { (RCC->APB2RSTR |= ( 1 << 12 ));	(RCC->APB2RSTR &= ~( 1 << 12 )); } while(0)	//Set bit and reset SPI1 Reset bit
#define RCC_SPI2_RST()				do { (RCC->APB1RSTR |= ( 1 << 14 ));	(RCC->APB2RSTR &= ~( 1 << 14 )); } while(0)	//Set bit and reset SPI2 Reset bit
#define RCC_SPI3_RST()				do { (RCC->APB1RSTR |= ( 1 << 15 ));	(RCC->APB2RSTR &= ~( 1 << 15 )); } while(0)	//Set bit and reset SPI3 Reset bit
#define RCC_SPI4_RST()				do { (RCC->APB2RSTR |= ( 1 << 15 ));	(RCC->APB2RSTR &= ~( 1 << 15 )); } while(0)	//Set bit and reset SPI4 Reset bit

#define GPIO_BASE_TO_CODE(x)		( (x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOD) ? 3 :\
									  (x == GPIOE) ? 4 :\
									  (x == GPIOF) ? 5 :\
									  (x == GPIOG) ? 6 : 0 )

#define ENABLE 						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define HIGH						SET
#define LOW							RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET


#include "stm32f303xx_driver_gpio.h"	//GPIO driver driver header file inclusion
#include "stm32f303xx_driver_spi.h"		//SPI driver header file inclusion

#endif /* INC_STM32F303XX_H_ */
