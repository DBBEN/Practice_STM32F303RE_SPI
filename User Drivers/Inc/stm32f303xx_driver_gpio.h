/*
 * stm32f303re_driver_gpio.h
 *
 *  Created on: Jun 11, 2020
 *      Author: Dave
 */

#ifndef INC_STM32F303XX_DRIVER_GPIO_H_
#define INC_STM32F303XX_DRIVER_GPIO_H_

#include "stm32f303xx.h"

typedef struct
{
	uint8_t pinNumber;	//GPIO pin number
	uint8_t pinMode;	//GPIO pin mode		------------	<@GPIO_PIN_MODES>
	uint8_t pinSpeed;	//GPIO pin speed	------------	<@GPIO_OP_SPEED>
	uint8_t pinPuPd;	//GPIO pull-up/pull-down	----	<@GPIO_PIN_PUPD>
	uint8_t pinOPType;	//GPIO pin output type	--------	<@GPIO_OP_TYPES>
	uint8_t pinAFMode;	//GPIO Alternate function mode
}GPIOconfig_Typedef;

typedef struct
{
	GPIO_Typedef *pGPIOx; 			//Holds the base address of the GPIO port
	GPIOconfig_Typedef GPIOconfig;	//Holds the GPIO pin configuration settings
}GPIO_Handle;

//----------------------------------------------
//		GPIO Pin Number Macros
#define GPIO_PIN0					0	//GPIO PORTx Pin 0
#define GPIO_PIN1					1	//GPIO PORTx Pin 1
#define GPIO_PIN2					2	//GPIO PORTx Pin 2
#define GPIO_PIN3					3	//GPIO PORTx Pin 3
#define GPIO_PIN4					4	//GPIO PORTx Pin 4
#define GPIO_PIN5					5	//GPIO PORTx Pin 5
#define GPIO_PIN6					6	//GPIO PORTx Pin 6
#define GPIO_PIN7					7	//GPIO PORTx Pin 7
#define GPIO_PIN8					8	//GPIO PORTx Pin 8
#define GPIO_PIN9					9	//GPIO PORTx Pin 9
#define GPIO_PIN10					10	//GPIO PORTx Pin 10
#define GPIO_PIN11					11	//GPIO PORTx Pin 11
#define GPIO_PIN12					12	//GPIO PORTx Pin 12
#define GPIO_PIN13					13	//GPIO PORTx Pin 13
#define GPIO_PIN14					14	//GPIO PORTx Pin 14
#define GPIO_PIN15					15	//GPIO PORTx Pin 15

//----------------------------------------------
//		@GPIO_PIN_MODES

#define GPIO_MODE_INPUT				0	//Input mode configuration
#define GPIO_MODE_OUTPUT			1	//Output mode configuration
#define GPIO_MODE_AF				2	//Alternate Functionality mode configuration
#define GPIO_MODE_ANALOG			3	//Analog mode configuration
//---- 		Interrupt modes --------
#define GPIO_IT_RISING				4	//Rising edge interrupt detection
#define GPIO_IT_FALLING				5	//Falling edge interrupt detection
#define GPIO_IT_RISING_FALLING		6	//Rising and Falling edge interrupt detection

//----------------------------------------------
//		@GPIO_OP_TYPES

#define GPIO_OP_TYPE_PUPL			0	//Push-Pull Output Type
#define GPIO_OP_TYPE_OD				1	//Open Drain Output Type

//----------------------------------------------
//		@GPIO_OP_SPEED

#define GPIO_OP_SPEED_LOW			0	//Low Output Speed
#define GPIO_OP_SPEED_MEDIUM		1	//Medium Output Speed
#define GPIO_OP_SPEED_HIGH			2	//High Output Speed

//----------------------------------------------
//		@GPIO_PIN_PUPD

#define GPIO_PIN_PUPD_NONE			0	//No Pull-up/Pull-down for GPIO pin
#define GPIO_PIN_PUPD_PU			1	//Pull-up for GPIO pin
#define GPIO_PIN_PUPD_PD			2	//Pull-down for GPIO pin


/*******************************************************************
 * 				API FUNCTION PROTOTYPES
 *******************************************************************/
//Enable GPIO Port Clock
void gpio_portClk(GPIO_Typedef *pGPIOx, uint8_t enOrDis);


void gpio_init(GPIO_Handle *pGPIOHandle);
void gpio_deInit(GPIO_Typedef *pGPIOx);


uint8_t gpio_readInputPin(GPIO_Typedef *pGPIOx, uint8_t pinNumber);
uint16_t gpio_readInputPort(GPIO_Typedef *pGPIOx);
void gpio_writeOutputPin(GPIO_Typedef *pGPIOx, uint8_t pinNumber, uint8_t value);

void gpio_digitalWrite(GPIO_Handle newGPIO, uint8_t value);
uint8_t gpio_digitalRead(GPIO_Handle newGPIO);

void gpio_writeOutputPort(GPIO_Typedef *pGPIOx, uint16_t value);
void gpio_toggleOutputPin(GPIO_Typedef *pGPIOx, uint8_t pinNumber);

void gpio_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void gpio_IRQInterruptConfig(uint8_t IRQNumber,uint8_t enOrDis);
void gpio_IRQhandling(uint8_t pinNumber);

#endif /* INC_STM32F303XX_DRIVER_GPIO_H_ */
