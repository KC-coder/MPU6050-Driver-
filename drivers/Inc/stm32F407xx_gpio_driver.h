/*
 * stm32F407xx_gpio_driver.h
 *
 *  Created on: 24-Aug-2020
 *      Author: MUKUL
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include"stm32F407xx.h"

typedef struct
{
	// it contains the required items for pin configuration
	// 1 byte is enough for configuration as max correct possible is 16

	uint8_t GPIO_PinNumber ;		/*<possible value from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode ;			/*<possible value from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed ;			/*<possible value from @GPIO_PIN_OUT_SPEED >*/
	uint8_t GPIO_PinPuPdControl ;	/*<possible value from @GPIO_PIN_OUT_TYPE >*/
	uint8_t GPIO_PinOPType ;		/*<possible value from @GPIO_PIN_PU_AND_PD_COUNTROL>*/
	uint8_t GPIO_PinAlFunMode ;
}GPIO_PinConfig_t;


/* GPIOx handle structure */
typedef struct
{
	GPIO_RegDef_t *pGPIOx ; 		 /*Pointer variable to  GPIO_RegDef_t of MCU header
	 	 	 	 	 	 			  *this is to hold the base address of gpio port to which pin belongs */

	GPIO_PinConfig_t GPIO_PinConfig;/*this is to hold pin configuration  settings  */

}GPIO_Handle_t;

/*======================================================================================
 *  SOME DRIVER SPECIFIC MACROS
 *======================================================================================
 */


/* @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN					0
#define GPIO_MODE_OUT					1
#define GPIO_MODE_ALTFUN				2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4	//For interrupt falling edge trigger
#define GPIO_MODE_IT_RT					5	//For interrupt rising edge trigger
#define GPIO_MODE_IT_RFT				6	//For interrupt rising & falling edge trigger

/* @GPIO_PIN_OUT_TYPE
 * GPIO pin output types
 */

#define GPIO_OP_TYPE_PP					0	//push-pull type
#define GPIO_OP_TYPE_OD					1	//open drain type

/* @GPIO_PIN_OUT_SPEED
 * GPIO pin possible output speed
 */

#define GPIO_SPEED_LOW					0
#define GPIO_SPEED_MEDIUM				1
#define GPIO_SPEED_FAST					2
#define GPIO_SPEED_HIGH					3

/*  @GPIO_PIN_PU_AND_PD_COUNTROL
 * GPIO pin pull-up and pull-down configuration
 */

#define GPIO_NO_PUPD					0
#define GPIO_PU							1
#define GPIO_PD							2

/* @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0						0
#define GPIO_PIN_1						1
#define GPIO_PIN_2						2
#define GPIO_PIN_3						3
#define GPIO_PIN_4						4
#define GPIO_PIN_5						5
#define GPIO_PIN_6						6
#define GPIO_PIN_7						7
#define GPIO_PIN_8						8
#define GPIO_PIN_9						9
#define GPIO_PIN_10						10
#define GPIO_PIN_11						11
#define GPIO_PIN_12						12
#define GPIO_PIN_13						13
#define GPIO_PIN_14						14
#define GPIO_PIN_15						15




/*===========================================================================================
 * 						APIs Supported by this driver file
 * 	For details related to API's refer function definition in .c file of the driver
 *==========================================================================================
 */

//GPIO_RegDef_t *pGPIOx : used to take the base address of the GPIO

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi);

/*
 * For installation and deinstallation
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInt(GPIO_RegDef_t *pGPIOx ); // for this we have a reset register in RCC section to reset a GPIO

/*
 * For data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);   // uint16_t as port is of 16pins,as we operating on whole port so no need of  pin number
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); // void as it is write
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * For IRQ configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);	//for interrupt ,this is used to config IRQ number, priority ,etc.

void GPIO_IRQHandling(uint8_t PinNumber);     				 // whenever an interrupt occurs user can call this to process the interrupt

void GPIO_IRQPriorityConfig (uint8_t IRQNumber , uint32_t IRQPriority);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
