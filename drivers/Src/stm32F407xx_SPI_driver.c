/*
 * stm32F407xx_SPI_driver.c
 *
 *  Created on: 01-Sep-2020
 *      Author: KAMAL CHOPRA
 */

#include"stm32F407xx.h"
#include"stm32F407xx_SPI_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);//here static is used to make these function
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);//Private
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


//SPI_RegDef_t	*pSPIx; : used to take the base address of the SPIx

/*
 * Peripheral clock setup
 */

/*****************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}else
		{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx ==SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}

	}
}



/*****************************************************************
 * @fn			- SPI_Init
 *
 * @brief		- In this function me configure SPI's CR1 and CR2
 * 				  register , regarding various SPI settings
 *
 * @param[in]	- Pointer to the SPI handle structure
 *
 * @return		- None
 *
 * @Note		- in this first we configure the setting in a temp variable
 * 				  then copy that variable in the CR register
 *
 *****************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//1.Configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//A.configuring the device mode
	tempreg |=  ( pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR );

	//B.congiguring the BUS configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDIMODE should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDIMODE should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDIMODE should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY should be set
		tempreg |=  (1<<SPI_CR1_RXONLY);

	}

	//C.configuring the clock speed

	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//D.configuring the DFF (Data frame format)
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);


	//E.configuring the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//F.configuring the CPAH
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPAH << SPI_CR1_CPHA);

	//G.configuring the SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);


	pSPIHandle->pSPIx->CR1 =  (tempreg); //CAN USE assignment operator


}


/*****************************************************************
 * @fn			- SPI_DeInt
 *
 * @brief		- This function de-initialize SPI peripherals
 *
 *
 * @param[in]	- Base address of the SPI peripheral
 *
 * @return		- None
 *
 * @Note		- here we have to reset all the registers of SPI
 * 				  To their reset value
 * 				  here we use RCC_AHB1RSTR register to reset SPIx
 *
 * @note 		- HERE WE SET THE REQUIRED BITS AND THEN CLEAR THOSE BITS
 * 				  As when we set the bits that SPI  is in reset state
 * 				  so we have to clear those  bits to remove it form reset state
 **
 *****************************************************************/

void SPI_DeInt(SPI_RegDef_t *pSPIx )
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

}








uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*****************************************************************
 * @fn			- SPI_SendData
 *
 * @brief		- This function is to send the data through SPI to external
 * 				  world
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- data to be send
 * @param[in]	- Size of data to be send
 *
 * @return		- None
 *
 * @Note		- To send data we write data in SPI_DR register
 * 				  from that register data is passed to TxBuffer
 * 				  before writing data to Txbuffer we have to check the
 * 				  Tx flag ofSPI_SR register
 *
 * @note		- this is blocking type , it does't return function call  till all data is
 * 				  is transfered
 * 				  it can even block the permanently due to some fault , se we need watchdog
 * 				  setup to prevent that
 *
 *****************************************************************/

//IN HAL LAYER THEY HAVE CREATED A TIMOUT , ARGUMENT IN WHICH WE SET THE TIME TILL WHICH FUCTION
//CAN WAIT TILL BUSY FLAG IS RESET
//REFER THIS ::: HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)

void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0) // this makes it a blocking call
	{
		//1. Wait till TXE is set

		/*uint16_t temp = 0;
		temp = pSPIx->SR;
		temp = temp & 0x02;
		while(temp = 0);
		I prefer this way of getting the flag
		as by using this other bits of SR register retain their value */

		while( SPI_GetFlagStatus(pSPIx , SPI_TXE_FLAG) == FLAG_RESET ); // this makes it a blocking call

		//2.check the DFF value
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF ))
		{
			//then 16 data format

			//1load data in SPI_DR
			pSPIx->DR =  *((uint16_t*)pTxBuffer );	// here i have type casted the pointer to get
						// 16 bit of data, as pointer was 8 bit it would have given 8 bit of data
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++; //incrementing the buffer so it points to next data , as we send 16 bit
					//of data so we have to type cast it so it points to data after 16 bits.
		}else
		{
			//8bit data format

			//1load data in SPI_DR
			pSPIx->DR =  *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


/*****************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0) // this makes it a blocking call
	{
		//1. Wait till TXE is set

		/*uint16_t temp = 0;
		temp = pSPIx->SR;
		temp = temp & 0x02;
		while(temp = 0);
		I prefer this way of getting the flag
		as by using this other bits of SR register retain their value */

		while( SPI_GetFlagStatus(pSPIx , SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET ); // this makes it a blocking call

		//2.check the DFF value
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF ))
		{
			//then 16 data format

			//1 load data from RXbuffer to  SPI_DR
			  *(uint16_t*)pRxBuffer =  pSPIx->DR ;	// here i have type casted the pointer to get
						// 16 bit of data, as pointer was 8 bit it would have given 8 bit of data
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++; //incrementing the buffer so it points to next data , as we send 16 bit
					//of data so we have to type cast it so it points to data after 16 bits.
		}else
		{
			//8bit data format

			//1load data in SPI_DR
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

}




/*****************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/




/*****************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void SPI_IRQPriorityConfig (uint8_t IRQNumber , uint32_t IRQPriority)
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
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given SPI peripheral
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- here we enable/disable the SPI peripheral to send/receive data
 *
 *****************************************************************/


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx ,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*****************************************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		- This function enables or disables the SSI bit of
 * 				  CR1
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- here we Enable and Disable the SSI bit of CR1
 * 				  -we need to enable SSi bit if software slave mangement is enabled
 * 				   as by setting this bit we make NSS +vcc internally and avoid
 * 				   multimaster error(MODF error)
 * 				  -in multimaster mode we have to make can make SSI = 0, As by making
 * 				  SSI = 0 we make NSS =0 and making NSS = 0 means some other master
 * 				  has taken over the bus .
 *
 *
 *****************************************************************/

void SPI_SSIConfig(SPI_RegDef_t *pSPIx ,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx ,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}

}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState =SPI_BUSY_IN_TX ;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1 , temp2;
	//first lets check for TXE
	/* check it in debuing if this stype of code is changing rest of the bits or not */
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);//to check if txe

	if( temp1 && temp2)//if both are true
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle); //not for user ,internal to driver as a helper
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
//incomplete , 1 more flag has to be checked

}


//some helper function implementations

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		//this function has to be impemented in the user application

	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

//we have to given week implementation of this call back , as implementation of this function depend upon
//user , so to prevent the error we use GCC ,week attribute

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
	//this function is mention here with week attribute as if user , does't implement this function then also
	//the complier would't issue an error , as implementation of this function is user dependent
}



