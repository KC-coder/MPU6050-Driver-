/*
 * stm32F407xx_gpio_driver.c
 * FOR MCU : STM32F407VGT6
 *
 *  Created on: 24-Aug-2020
 *      Author: KAMAL CHOPRA
 */

#include"stm32F407xx.h"
#include"stm32F407xx_gpio_driver.h"


//GPIO_RegDef_t *pGPIOx : used to take the base address of the GPIO

/*
 * Peripheral clock setup
 */

/*****************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	 {
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}

	}
}

/*
 * For installation and deinstallation
 */
/*****************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initialize GPIO peripherals
 *
 * @param[in]	- Pointer to GPIO Handle structure
 * @param[in]	- none
 *
 * @return		- None
 *
 * @Note		- In this we have to initialize GPIO pin and GPIO port
 * 				  Here we configure the Mode, OutType, Speed,
 * 				  AltFunction, PuPd settings
 *
 *@concept		- here function receive values in form of  gpio handle
 *@concept		  structure,to set mode we take the mode value form this
 *@concept		  at the  pin number, in a varablr temp on the required bits
 *@concept		  then get address of required of mode register from handle fun
 *@concept		  and and copy the temp value to it .
 *
 *@note2		- we have cleared bits first as we are doing OR operation
 *@note2		  so if the bits are cleared then only it could be performed
 *@note2		  correctly.
 *****************************************************************/


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0 ;
	//1. Configure the Mode of GPIO pin

	if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// when non interrupt mode
		temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<(2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber )); // clearing bits

		pGPIOHandle->pGPIOx->MODER |= temp; // to store pin mode value in the appropriate register we use OR so
											// of bits remain same , if we equate of bits gets changed


	}else
	{
		// when interrupt mode

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.CONFIGURE FTSR

			EXTI ->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//WE are clearing the RTSR bit as we only want falling edge detection
			EXTI->RTSR &=~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.configure RTSR

			EXTI ->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//WE are clearing the FTSR bit as we only want rising edge detection
			EXTI->FTSR  &=~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.configure BOTH RTSR AND FTSR as we want both rising edge and
			//falling edge detection
			EXTI ->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR  |=( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);//to return code used function not macro
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode <<(temp2 *4);


		//3. Enable the exti interrupt delivery using IMR

		EXTI->IMR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}

	temp = 0;
	//2. Configure the Speed

	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber )); // clearing bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the PuPd settings

	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the OutType

	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the AltFunction
	if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		// to configure alternate function
		uint32_t temp1 = 0 ;
		uint32_t temp2 = 0 ;

		temp1 = ((pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber ) / 8 ); // for AF0 or AF1
		temp2 = ((pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber ) % 8 ); // for selecting bits of AF

		pGPIOHandle ->pGPIOx->AFR[temp1] &= ~(0xF  << (4*temp2) );
		pGPIOHandle ->pGPIOx->AFR[temp1] |= (pGPIOHandle ->GPIO_PinConfig.GPIO_PinAlFunMode  << (4*temp2) );


	}

}

/*****************************************************************
 * @fn			- GPIO_DeInt
 *
 * @brief		- This function de-initialize GPIO peripherals
 *
 *
 * @param[in]	- Base address of the GPIO poart
 *
 *
 * @return		- None
 *
 * @Note		- here we have to reset all the registers of gpio port
 * 				  which are supplied to this function .
 * 				  here we use RCC_AHB1RSTR register to reset gpio ports
 *
 * @note 		- HERE WE SET THE REQUIRED BITS AND THEN CLEAR THOSE BITS
 * 				  As when we set the bits that port is in reset state
 * 				  so we have to clear that bit to remove it form reset state
 *
 *****************************************************************/


void GPIO_DeInt(GPIO_RegDef_t *pGPIOx ) // for this we have a reset register in RCC section to reset a GPIO
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}


/*
 * For data read and write
 */

/*****************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function is for reading data from a pin
 * 			  	  it reads data through GPIO port input data register
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- gpio pin number
 *
 * @return		- Content of the input data
 * 				- that is 0 or 1
 *
 * @Note		- None
 *
 *****************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;

}


/*****************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This function is for reading data from a GPIO PORT
 * 			  	  it reads data through: GPIO port input data register
 *
 * @param[in]	- Base address of the GPIO peripheral
 *
 *
 * @return		- Content of the input data
 * 				- that is 0 or 1
 *
 * @Note		- None
 *
 *****************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) // uint16_t as port is of 16pins,as we operating on whole port so no need of  pin number
{
	uint16_t value;
	value = (uint16_t )pGPIOx->IDR ;

	return value;
}


/*****************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- This function writes value on a specific
 * 			  	  output pin
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- gpio pin number
 * @param[in]	- gpio pin value that is to be written
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) // void as it is write
{
	if(Value == GPIO_PIN_SET)
		{
			/* Write 1 to the output data register at the bit field corresponding to the pin number */
			pGPIOx->ODR |= ( 1 << PinNumber );//set pin
		}
		else
		{
			/* Write 0 to the output data register at the bit field corresponding to the pin number */
			pGPIOx->ODR &= ~( 1 << PinNumber );	//Clear pin
		}
}


/*****************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- This function writes value on a specific
 * 			      output port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Value (Set/Reset Macro)
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*****************************************************************
 * @fn			-  GPIO_ToggleOutputPin
 *
 * @brief		- This function is used to toggle the specific GPIO output pin
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- gpio pin number
 *
 * @return		- None
 *
 * @Note		- uses XOR operation
 *
 *****************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



/*
 * For IRQ configuration and ISR handling
 */

/*****************************************************************
 * @fn			- GPIO_IRQInterruptConfig
 *
 * @brief		- This function is to configure NVIC of the processor
 * 					for the interrupt handling
 *
 * @param[in]	- IRQNumber
 * @param[in]	- enableordisable
 *
 * @return		- None
 *
 * @Note		- what we configure here is processor specific,
 * 				  here we configure registers of NVIC of the processor
 * 				  detalis of register are in the processor generic guide
 * 				  Interrupt Set-enable Registers
 *
 *****************************************************************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program NVIC_ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber< 64)
		{
			//program NVIC_ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber< 96)
		{
			//program NVIC_ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}

	}else
	{
		if(IRQNumber <= 31)
		{
			//program NVIC_ICER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber< 64)
		{
			//program NVIC_ICER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program NVIC_ICER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	}

	/*
	 * no need to implement other NVIC_ISER and NVIC_ICER Register as in MCU
	 * only 82 IRQ numbers are implemented
	 */
}


/*****************************************************************
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- This function is to set the interrupt priority in the
 * 				  NVIC of the processor
 *
 * @param[in]	- IRQNumber
 * @param[in]	- IRQPriority
 *
 * @return		- None
 *
 * @Note		- what we configure here is processor specific,
 * 				  here we configure registers of NVIC of the processor
 * 				  detalis of register are in the processor generic guide
 * 				  Interrupt Set-enable Registers
 *
 *****************************************************************/

void GPIO_IRQPriorityConfig (uint8_t IRQNumber , uint32_t IRQPriority)
{
	//1.finding IPR Register to be configured
	uint8_t iprx = (IRQNumber / 4) ;
	//2.finding the section of IRQ register
	uint8_t iprx_section = IRQNumber % 4 ;

	uint8_t  shift_amount = (8*iprx_section) +  (8 - NO_PR_BITS_IMPLEMENTED );
	//here, 8*iprx_section as each section is of 8 bits

	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
	//here, iprx * 4 as each register is of 4 bytes


}

/*****************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- This function we clear the PR register of EXTI line
 *
 * @param[in]	- gpio pin number
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/


void GPIO_IRQHandling(uint8_t PinNumber) // whenever an interrupt occurs user can call this to process the interrupt
{
	//clear the PR register corresponding to the pin
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear //dout check again
		EXTI->PR |= (1 << PinNumber);

	}
}
