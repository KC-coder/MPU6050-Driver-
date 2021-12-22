/*
 * stm32F407xx_TIM_driver.c
 *
 *  Created on: 06-Sep-2020
 *      Author: MUKUL
 */


#include"stm32F407xx.h"
#include"stm32F407xx_TIM_driver.h"

/*==========================================================================================================
 * FUNCTIONS IMPLIMENTED IN THIS FILE
 *
 * - void TIM_PeriClockControl(TIM_RegDef_t *pTIMx , uint8_t EnorDi)
 * - void TIM_BASE_Init(TIM_Handle_t *pTIMHandle)
 * - void TIM_COUNTER_EnOrDi(TIM_RegDef_t *pTIMx , uint8_t EnOrDi)
 * - void TIMx_IC_Config(TIM_RegDef_t	*pTIMx , TIM_IC_Init_Handle_t *pTimxHandle )
 * - void TIMx_IC_EnOrDi(TIM_RegDef_t*pTIMx , uint8_t EnOrDi , uint8_t ChannelNumber)
 * - void TIMx_IC_WithIT_EnOrDi(TIM_RegDef_t*pTIMx , uint8_t EnOrDi ,uint8_t ChannelNumber)
 * - uint32_t TIM_IC_Read(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber)
 * - void TIMx_PWM_IC_Config(TIM_RegDef_t*pTIMx , uint8_t ChannelNumber)
 * - void TIM_OC_Int(TIM_OC_InitTypeDef *pConfigOC, TIM_RegDef_t *pTIMx , uint8_t ChannelName)
 * - void TIM_OC_EnOrDi(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber , uint8_t EnOrDi)
 * - void TIM_OC_EnOrDi_WithIT(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber , uint8_t EnOrDi)
 * - void TIM_PWM_Init(TIM_Handle_t *pTimHandle, TIM_OC_InitTypeDef *pConfigOC)
 * - void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
 * - void TIM_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
 * - void TIM_BASIC_IRQ_HANDLER(TIM_RegDef_t *pTIMx)
 * - void TIM_ClearFlag(TIM_RegDef_t *pTIMx, uint16_t StatusFlagName)
 * - uint32_t TIM_IC_SpecialMode(TIM_RegDef_t*pTIMx , TIM_IC_Init_Handle_t *pTimxHandle , TIM_ClockConfigTypeDef *pClockConfig )
 *
 *
 *
 *
 *
 *
 *	ENCODER MODE OF TIMER IS NOT COVERED IN THIS DRIVER
 */

/*
 * Peripheral clock setup
 */

/*****************************************************************
 * @fn			- TIM_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given TIM peripheral registers
 *
 * @param[in]	- Base address of the TIM peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- here we configure the RCC register to start/Stop the peripheral clock
 * *
 *****************************************************************/


void TIM_PeriClockControl(TIM_RegDef_t *pTIMx , uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		if(pTIMx == TIM1)
		{
			TIM1_PCLK_EN();
		}else if(pTIMx == TIM2)
		{
			TIM2_PCLK_EN();
		}else if(pTIMx == TIM3)
		{
			TIM3_PCLK_EN();
		}else if(pTIMx == TIM4)
		{
			TIM4_PCLK_EN();
		}else if(pTIMx == TIM5)
		{
			TIM5_PCLK_EN();
		}else if(pTIMx == TIM6)
		{
			TIM6_PCLK_EN();
		}else if(pTIMx == TIM7)
		{
			TIM7_PCLK_EN();
		}else if(pTIMx == TIM8)
		{
			TIM8_PCLK_EN();
		}else if(pTIMx == TIM9)
		{
			TIM9_PCLK_EN();
		}else if(pTIMx == TIM10)
		{
			TIM10_PCLK_EN();
		}else if(pTIMx == TIM11)
		{
			TIM11_PCLK_EN();
		}else if(pTIMx == TIM12)
		{
			TIM12_PCLK_EN();
		}else if(pTIMx == TIM13)
		{
			TIM13_PCLK_EN();
		}else if(pTIMx == TIM14)
		{
			TIM14_PCLK_EN();
		}
	}else
	 {
		if(pTIMx == TIM1)
		{
			TIM1_PCLK_DN();
		}else if(pTIMx == TIM2)
		{
			TIM2_PCLK_DN();
		}else if(pTIMx == TIM3)
		{
			TIM3_PCLK_DN();
		}else if(pTIMx == TIM4)
		{
			TIM4_PCLK_DN();
		}else if(pTIMx == TIM5)
		{
			TIM5_PCLK_DN();
		}else if(pTIMx == TIM6)
		{
			TIM6_PCLK_DN();
		}else if(pTIMx == TIM7)
		{
			TIM7_PCLK_DN();
		}else if(pTIMx == TIM8)
		{
			TIM8_PCLK_DN();
		}else if(pTIMx == TIM9)
		{
			TIM9_PCLK_DN();
		}else if(pTIMx == TIM10)
		{
			TIM10_PCLK_DN();
		}else if(pTIMx == TIM11)
		{
			TIM11_PCLK_DN();
		}else if(pTIMx == TIM12)
		{
			TIM12_PCLK_DN();
		}else if(pTIMx == TIM13)
		{
			TIM13_PCLK_DN();
		}else if(pTIMx == TIM14)
		{
			TIM14_PCLK_DN();
		}

	}
}

/*function to int the timer base unit
 *
 */
/*****************************************************************
 * @fn			- TIM_BASIC_Init
 *
 * @brief		- This function is to configure the registers of base unit of
 * 				  TIM peripheral
 *
 * @param[in]	- TIM handle structure
 *
 * @return		- None
 *
 * @Note		-  THIS FUNCTION HAS TO BE CALLED IN ALMOST EVERY MODE OF TIMER
 * 				   here we  configure the
 * 					- auto reload register(TIMx_ARR)
 *					- timer clock prescaler(TIMx_PRC)
 *					- timer count register(TIMx_CNT)
 *					-timer count mode(TIMx_CR1)
 *					- ALSO ENABLE OR DISABLE THE ARR BUFFER
 *****************************************************************/



void TIM_BASE_Init(TIM_Handle_t *pTIMHandle)
{
/*

	//TO LOAD THE COUNT, that is the value from which we want our counter to start
	 pTIMHandle->pTIMx->CNT  = pTIMHandle->TIMConfig.Period;
*/

	 //LOADING AUTO RELAOD REGISTER
	 pTIMHandle->pTIMx->ARR = pTIMHandle->TIMConfig.Period;
	  /* Set the auto-reload preload */

	 if(pTIMHandle->TIMConfig.AutoRelaodBuffer == TIM_ARR_BUFFER_EN)
	 {
		 pTIMHandle->pTIMx->CR1 = (ENABLE << TIMx_CR1_ARPE);
	 }


	 //to set the prescalar value
	 pTIMHandle->pTIMx->PSC = pTIMHandle->TIMConfig.Prescaler;

	 //to set counter mode
	 if(pTIMHandle->TIMConfig.CounterMode ==  (TIM_COUNTERMODE_UP || TIM_COUNTERMODE_DOWN))
	 {
		 pTIMHandle->pTIMx->CR1 &= ~(1 << TIMx_CR1_DIR);
		 pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIMConfig.CounterMode << TIMx_CR1_DIR);
	 }else
	 {
		 pTIMHandle->pTIMx->CR1 &= ~(3 << TIMx_CR1_CMS);
		 pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIMConfig.CounterMode << TIMx_CR1_CMS);
	 }

	  /* Generate an update event to reload the Prescaler
	     and the repetition counter (only for advanced timer) value immediately */
	 pTIMHandle->pTIMx->EGR = (1<<TIMx_EGR_UG);




}

/*****************************************************************
 * @fn			- TIM_COUNTER_EnOrDi
 *
 * @brief		- This function is to Enable the  TIMER to counter
 *
 * @param[in]	- Timer base address
 *
 * @return		- None
 *
 * @Note		- Here we enable the timer counter, as when we supply clock to the counter of timer
 * 				  counter will only start counting when the CEN bit of TIM_CR1 is SET
 * 				  so in this function we can enable and disbal the counter of the timer
 *
 *****************************************************************/


void TIM_COUNTER_EnOrDi(TIM_RegDef_t *pTIMx , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pTIMx->CR1 |= (1 << TIMx_CR1_CEN);

	}else
	{
		pTIMx->CR1 &= ~(1 << TIMx_CR1_CEN);
	}

}
/*========================================================================================
 * 			INPUT CAPTURE FUNCTIONS
 *========================================================================================*/

/* ****************************************************************
 * @fn			- TIMx_IC_Config
 *
 * @brief		- This function is used to configure the channel of the timer
 * 				  to capture input
 *
 * @param[in]	- Timer base address
 * @param[in]	- Timer input capture handle structure
 *
 * @return		- None
 *
 * @Note		- here we configure the selected channel for input capture
 * 				  -input capture Prescaler
 * 				  -input capture filter
 * 				  -active input ( i.e IC1 can have input as TI1 or TRC or TI2 )
 * 				  -input trigger polarity
 * 				  -enable the input capture register, so after every transition the
 * 				   counter value is latched in the required register(for this we have made seprate function )
 *
 *****************************************************************/

void TIMx_IC_Config(TIM_RegDef_t	*pTIMx , TIM_IC_Init_Handle_t *pTimxHandle )
{

	uint32_t temp1 = pTimxHandle->TIM_ChannelNumber;

	if(temp1 == TIM_IC_Channel_1 )
	{
		//1.to configure the direction of the channel and its active  input
		pTIMx->CCMR1 &= ~(3 << TIMx_CCMR1_CC1S);
		pTIMx->CCMR1 |= (pTimxHandle->ICSelection << TIMx_CCMR1_CC1S);

		//2.to configure the  Input capture  filter
		pTIMx->CCMR1 &= ~(15 << TIMx_CCMR1_IC1F);
		pTIMx->CCMR1 |= (pTimxHandle->IC_filter << TIMx_CCMR1_IC1F);

		//3.to configure the input capture polarity
		pTIMx->CCER &= ~(1<< TIMx_CCER_CC1P);
		pTIMx->CCER &= ~(1 << TIMx_CCER_CC1NP);
		if(pTimxHandle->ICPolarity == TIM_ICPOLARITY_BOTHEDGE)
		{
			pTIMx->CCER	|= (1 << TIMx_CCER_CC1P);
			pTIMx->CCER	|= (1 << TIMx_CCER_CC1NP);

		}else
		{

			pTIMx->CCER	|= (pTimxHandle->ICPolarity << TIMx_CCER_CC1P);

		}

		//4.To configure the input capture  prescaler
		pTIMx->CCMR1 &= ~(3 << TIMx_CCMR1_IC1PSC);
		pTIMx->CCMR1 |= (pTimxHandle->ICPrescaler << TIMx_CCMR1_IC1PSC);

		//5.To Capture the counter value into the input capture/compare register


	}else if(temp1 == TIM_IC_Channel_2)
	{
		pTIMx->CCMR1 &= ~(15 << TIMx_CCMR1_IC2F);
		pTIMx->CCMR1 |= (pTimxHandle->IC_filter << TIMx_CCMR1_IC2F);

		pTIMx->CCMR1 &= ~(3 << TIMx_CCMR1_CC2S);
		pTIMx->CCMR1 |= (pTimxHandle->ICSelection << TIMx_CCMR1_CC2S);

		pTIMx->CCER &= ~(1 << TIMx_CCER_CC2P);
		pTIMx->CCER &= ~(1 << TIMx_CCER_CC2NP);
		if(pTimxHandle->ICPolarity == TIM_ICPOLARITY_BOTHEDGE)
		{
			pTIMx->CCER	|= (1 << TIMx_CCER_CC2P);
			pTIMx->CCER	|= (1 << TIMx_CCER_CC2NP);
		}else
		{

			pTIMx->CCER	|= (pTimxHandle->ICPolarity << TIMx_CCER_CC2P);
		}

		pTIMx->CCMR1 &= ~(3 << TIMx_CCMR1_IC2PSC);
		pTIMx->CCMR1 |= (pTimxHandle->ICPrescaler << TIMx_CCMR1_IC2PSC);

		//capture of the counter value into the input capture/compare register


	}else if(temp1 == TIM_IC_Channel_3)
	{
		pTIMx->CCMR2 &= ~(15 << TIMx_CCMR2_IC3F);
		pTIMx->CCMR2 |= (pTimxHandle->IC_filter << TIMx_CCMR2_IC3F);

		pTIMx->CCMR2 &= ~(3  << TIMx_CCMR2_CC3S);
		pTIMx->CCMR2 |= (pTimxHandle->ICSelection << TIMx_CCMR2_CC3S);

		pTIMx->CCER &= ~(1 << TIMx_CCER_CC3P);
		pTIMx->CCER &= ~(1 << TIMx_CCER_CC3NP);
		if(pTimxHandle->ICPolarity == TIM_ICPOLARITY_BOTHEDGE)
		{
			pTIMx->CCER	|= (1 << TIMx_CCER_CC3P);
			pTIMx->CCER	|= (1 << TIMx_CCER_CC3NP);
		}else
		{

			pTIMx->CCER	|= (pTimxHandle->ICPolarity << TIMx_CCER_CC3P);
		}

		pTIMx->CCMR2 &= ~(3 << TIMx_CCMR2_IC3PSC);
		pTIMx->CCMR2 |= (pTimxHandle->ICPrescaler << TIMx_CCMR2_IC3PSC);

		//capture of the counter value into the input capture/compare register


	}else if(temp1 == TIM_IC_Channel_4)
	{
		pTIMx->CCMR2 &= ~(15 << TIMx_CCMR2_IC4F);
		pTIMx->CCMR2 |= (pTimxHandle->IC_filter << TIMx_CCMR2_IC4F);

		pTIMx->CCMR2 &= ~(3 << TIMx_CCMR2_CC4S);
		pTIMx->CCMR2 |= (pTimxHandle->ICSelection << TIMx_CCMR2_CC4S);

		pTIMx->CCER &= ~(1 << TIMx_CCER_CC4P);
		pTIMx->CCER &= ~(1<< TIMx_CCER_CC4P);
		if(pTimxHandle->ICPolarity == TIM_ICPOLARITY_BOTHEDGE)
		{
			pTIMx->CCER	|= (1 << TIMx_CCER_CC4P);
			pTIMx->CCER	|= (1 << TIMx_CCER_CC4NP);
		}else
		{

			pTIMx->CCER	|= (pTimxHandle->ICPolarity << TIMx_CCER_CC4P);
		}

		pTIMx->CCMR2 &= ~(3 << TIMx_CCMR2_IC4PSC);
		pTIMx->CCMR2 |= (pTimxHandle->ICPrescaler << TIMx_CCMR2_IC4PSC);


		//capture of the counter value into the input capture/compare register


	}

}

/*****************************************************************
 * @fn			-  TIMx_IC_EnOrDi
 *
 * @brief		- This function is to enable and disable input capture of the given channel
 *
 * @param[in]	- Timer base address
 *  @param[in]	- Channel number
 * @param[in]	- Enable or disable
 *
 *
 *
 * @Note		- After calling the input capture config function this function has to be called
 * 				  as this function enbale the CCRx register for input capture,
 *
 *
 * 				- The counter has to be enabled and the timer Base unit has to be configured before
 * 				  we could capture the input as when the trigger signal of the selected polarity occurs
 * 				  on the configured channel the CCRs capture's the CNT (counter) value.
 *

 *
 *****************************************************************/

void TIMx_IC_EnOrDi(TIM_RegDef_t*pTIMx , uint8_t EnOrDi , uint8_t ChannelNumber)
{
	if(ChannelNumber == TIM_IC_Channel_1)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC1E);
		}

	}else if(ChannelNumber == TIM_IC_Channel_2)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC2E);
		}


	}else if(ChannelNumber == TIM_IC_Channel_3)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC3E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC3E);
		}


	}else if(ChannelNumber == TIM_IC_Channel_4)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC4E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC3E);
		}


	}

}

/*****************************************************************
 * @fn			- TIMx_IC_WithIT_EnOrDi
 *
 * @brief		-This function is to enable and disable input capture of the given channel with
 * 				 interrupt
 *
 * @param[in]	- Timer base address
 *  @param[in]	- Channel number
 * @param[in]	- Enable or disable
 *
 *
 *
 * @Note		- After calling the input capture config function this function has to be called
 * 				  as this function enbale the CCRx register for input capture,
 *
 * 				- The counter has to be enabled and the timer Base unit has to be configured before
 * 				  we could capture the input as when the trigger signal of the selected polarity occurs
 * 				  on the configured channel the CCRs capture's the CNT (counter) value.
 * 				- This function also enable the interrupt for given channel so whenever the capture ours
 * 				  an interrupt triggers and CCxIF flag is set.
 *

 *
 *****************************************************************/
void TIMx_IC_WithIT_EnOrDi(TIM_RegDef_t*pTIMx , uint8_t EnOrDi ,uint8_t ChannelNumber)
{
	if(ChannelNumber == TIM_IC_Channel_1)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC1IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC1E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC1IE);
		}

	}else if(ChannelNumber == TIM_IC_Channel_2)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC2IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC2E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC2IE);
		}


	}else if(ChannelNumber == TIM_IC_Channel_3)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC3E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC3IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC3E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC3IE);
		}


	}else if(ChannelNumber == TIM_IC_Channel_4)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC4E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC4IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC4E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC4IE);
		}


	}
}



/*****************************************************************
 * @fn			- TIM_IC_Read
 *
 * @brief		- This function is used to read the value captured by from input capture channel
 *
 * @param[in]	- Timer base address
 * @param[in]	- Timer ChannelNumber
 *
 * @return		- the  captured value of the selected channel
 *
 * @Note		- here we have to select the channel whose captured
 * 				  input we want to read, i.e input captured
 * 				  selected trigger edge
 *

 *
 *****************************************************************/

uint32_t TIM_IC_Read(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber)
{
	uint32_t Readvalue = 0;
	if(ChannelNumber == TIM_IC_Channel_1 )
	{
		Readvalue =  pTIMx->CCR1;
		return Readvalue;

	}else if(ChannelNumber == TIM_IC_Channel_2)
	{
		Readvalue =  pTIMx->CCR2;
		return Readvalue;

	}else if(ChannelNumber == TIM_IC_Channel_3)
	{
		Readvalue =  pTIMx->CCR3;
		return Readvalue;

	}else if(ChannelNumber == TIM_IC_Channel_4)
	{
		Readvalue =  pTIMx->CCR4;
		return Readvalue;
	}
	return 0;


}

/*****************************************************************
 * @fn			- TIMx_PWM_IC_Config
 *
 * @brief		- This function is to configure the timer read the PWM applied on the input channel
 *
 *
 * @param[in]	- TIMER BASE Address
 * @param[in]	- Channel Number
 *
 *
 * @return		- None
 *
 * @Note		- in this function we configure the channel to read the PWM applied
 * 				  in this we use single channel and two CCRx register
 * 				  one CCRx for duty cycle and other CCRx for the period of the PWM
 *
 * 				- also we use Reset mode of Slave controller so whenever a trigger pulse occur the CNT resets
 * 				  as we have used slave controller so we can only use channel 1 or channel 2 for this configuration
 * 				  as only IC signals of these channel are mapped on the slave controller
 *
 * 				- Example : we have used channel 1 then on the first rising edge CCR1 captures the CNT value
 * 							and on the falling edge the CCR2 captures the CNT value the difference between
 * 							CCR2 and CCR1 gives us the duty cycle of PWM
 * 							Then on the next rising edge the CCR1 again captures the CNT value
 * 							now the difference in the  previous CCR1 value and new CCR1 value give us the
 * 							period of the PWM
 *

 *
 *****************************************************************/
void TIMx_PWM_IC_Config(TIM_RegDef_t*pTIMx , uint8_t ChannelNumber)
{
	if (!(ChannelNumber == TIM_IC_Channel_1 || ChannelNumber == TIM_IC_Channel_2))
	{
		TIMx_Error();
	}

	if( ChannelNumber == TIM_IC_Channel_1 )
	{
		pTIMx->CCMR1 |= (TIM_ICSELECTION_DIRECTTI  << TIMx_CCMR1_CC1S);
		pTIMx->CCMR1 |= (TIM_ICSELECTION_INDIRECTTI  << TIMx_CCMR1_CC2S);
		pTIMx->CCER  |= (TIM_ICPOLARITY_RISING << TIMx_CCER_CC1P);
		pTIMx->CCER  |= (TIM_ICPOLARITY_FALLING	 << TIMx_CCER_CC2P);
		pTIMx->SMCR  |= (5 << TIMx_SMCR_TS);
		pTIMx->CCER  |=(ENABLE << TIMx_CCER_CC1E);
		pTIMx->CCER  |=(ENABLE << TIMx_CCER_CC2E);

	}else
	{
		pTIMx->CCMR1 |= (TIM_ICSELECTION_INDIRECTTI  << TIMx_CCMR1_CC1S);
		pTIMx->CCMR1 |= (TIM_ICSELECTION_DIRECTTI  << TIMx_CCMR1_CC2S);
		pTIMx->CCER  |= (TIM_ICPOLARITY_RISING << TIMx_CCER_CC2P);
		pTIMx->CCER  |= (TIM_ICPOLARITY_FALLING	 << TIMx_CCER_CC1P);
		pTIMx->SMCR  |= (6 << TIMx_SMCR_TS);
		pTIMx->CCER  |= (ENABLE << TIMx_CCER_CC1E);
		pTIMx->CCER  |= (ENABLE << TIMx_CCER_CC2E);


	}
	pTIMx->SMCR |= (TIM_SLAVEMODE_RESET << TIMx_SMCR_SMS);

}

/*===========================================================================================
 * 			FUNCTIONS FOR TIMER OUTPUT MODES
 * ===========================================================================================
 */
/*
 output compare mode is :
 * To control an output waveform, or to indicate when a period of time has elaps
*/

/****************************************************************
 * @fn			- TIM_OC_Int
 *
 * @brief		- this function is to configure the output capture mode of the timer
 *
 * @param[in]	- Timer base address
 *  @param[in]	- Timer output capture register
 *
 *
 * @return		- NONE
 *
 * @Note		- Very usefull function the configure the output of a timer's channel
 *
 * 				- Before calling this function  select the clock  and configure the ARR and CCRx registers
 * 					( i.e call the timer clock config and timer base int functions )
 *
 * 				- This function is to configure the timer channel  in output compare mode
 * 				  i.e we want the timer pin to produce output when CNT and CCRx value matches
 * 				  as when they match the channel output value changes according to the output compare mode
 * 				  selected  by the user
 *
 * 				- after calling this funtion  set the CNT bit of TIMx_CR1 to start the counter
 *
 * 				-configure the CCxIE BIT OR CCxDE BIT  if we want interrupt or DMA request
 * 				 or else we can always go for polling by constantly checking the flag register
 *
 * 				- we can use this function to create different waveformes at the output of the channel by
 * 				  manuplating the CCRx and ARR values a s per the requiements
 * 				  we can also generate PWM but for the PWM we have a special function
 *
 * 				-
 * @FunctionFlow -  Select the channel
 * 					1) Configure the channel as output CCxS of CCMRx
 * 					2) Configure the Channel's output mode
 * 					3) Configure the Polarity of output mode CCxP of CCER (i.e it can be active high or active low )
 * 					4) Enable to Capture compare register of the Channel
 * 					5) Enable the Preload register of CCRx
 * 					6) set the value to be compared in CCRx
 *
 *					MOdes for the output channel:
 *					1) Frozen -The comparison between the output compare register TIMx_CCR1 and the
						counter TIMx_CNT has no effect on the outputs.
 *					2)Set channel x to active level on match. OCxREF signal is forced high when the counter
						TIMx_CNT matches the capture/compare register.
 *					3)Set channel x to inactive level on match. OCxREF signal is forced low when the
						counter TIMx_CNT matches the capture/compare register.
 *					4)Toggle - OCxREF toggles when TIMx_CNT=TIMx_CCRx.
 *					5)Force inactive level - OCxREF is forced low.
 *					6)Force active level - OCxREF is forced high.
 ****************************************************************
*/

void TIM_OC_Int(TIM_OC_InitTypeDef *pConfigOC, TIM_RegDef_t *pTIMx , uint8_t ChannelName)
{
	//configure the Input capture filter
	uint8_t polaritycheck = 0 ;

	//TO CONFIGURE THE POLARITY
	if( (pConfigOC->OC_Mode == TIM_OC_MODE_Active )	|| ( pConfigOC->OC_Mode == TIM_OC_MODE_InActive ) || ( pConfigOC->OC_Mode == TIM_OC_MODE_Toggle )   )
	{
		polaritycheck = 1 ;
	}

	switch (ChannelName)
	{
		case TIM_OC_Channel_1:
		{
			//1.configure the pin as ouput
			pTIMx->CCMR1 &= ~(1 << TIMx_CCMR1_CC1S);
			//2.configure the ouput mode
			pTIMx->CCMR1 &= ~(7 << TIMx_CCMR1_OC1M);
			pTIMx->CCMR1 |= (pConfigOC->OC_Mode << TIMx_CCMR1_OC1M);

			if(polaritycheck)
			{
				pTIMx->CCER &= ~(1 << TIMx_CCER_CC1P);
				pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC1P);
			}
			//3.Disabling the preload register
			pTIMx->CCMR1 &= ~(TIM_OC_PRELOAD_DISABLED << TIMx_CCMR1_OC1PE);
			//4.capture of the counter value into the input capture/compare register for compare
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);
			//5.configure the pulse
			pTIMx->CCR1 = pConfigOC->Pulse;


		}
		break;
		case TIM_OC_Channel_2:
		{
			pTIMx->CCMR1 &= ~(1 << TIMx_CCMR1_CC2S);

			pTIMx->CCMR1 &= ~(7 << TIMx_CCMR1_OC2M);
			pTIMx->CCMR1 |= (pConfigOC->OC_Mode << TIMx_CCMR1_OC2M);
			if(polaritycheck)
			{
				pTIMx->CCER &= ~(1 << TIMx_CCER_CC2P);
				pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC2P);
			}

			pTIMx->CCMR1 &= ~(TIM_OC_PRELOAD_DISABLED << TIMx_CCMR1_OC2PE);
			//capture of the counter value into the input capture/compare register
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);
			pTIMx->CCR2 = pConfigOC->Pulse;

		}
		break;
		case TIM_OC_Channel_3:
		{
			pTIMx->CCMR2 &= ~(1 << TIMx_CCMR2_CC3S);
			pTIMx->CCMR2 &= ~(7<< TIMx_CCMR2_OC3M);
			pTIMx->CCMR2 |= (pConfigOC->OC_Mode << TIMx_CCMR2_OC3M);
			if(polaritycheck)
			{
				pTIMx->CCER &= ~(1 << TIMx_CCER_CC3P);
				pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC3P);
			}

			pTIMx->CCMR2 &= ~(TIM_OC_PRELOAD_DISABLED << TIMx_CCMR2_OC3PE);
			//capture of the counter value into the input capture/compare register
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC3E);
			pTIMx->CCR3 = pConfigOC->Pulse;

		}
		break;
		case TIM_OC_Channel_4:
		{
			pTIMx->CCMR2 &= ~(1 << TIMx_CCMR2_CC4S);
			pTIMx->CCMR2 &= ~(7 << TIMx_CCMR2_OC4M);
			pTIMx->CCMR2 |= (pConfigOC->OC_Mode << TIMx_CCMR2_OC4M);
			if(polaritycheck)
			{
				pTIMx->CCER &= ~(1 << TIMx_CCER_CC4P);
				pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC4P);
			}

			pTIMx->CCMR2 &= ~(TIM_OC_PRELOAD_DISABLED << TIMx_CCMR1_OC1PE);
			//capture of the counter value into the input capture/compare register
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC4E);
			pTIMx->CCR4 = pConfigOC->Pulse;

		}
		break;
		//NOW SET CEN control bit-field of CR1 to enable the counter for this we use
		//void TIM_BASIC_ENABLE(TIM_RegDef_t *pTIMx)



	}

}
/*****************************************************************
 * @fn			- TIM_OC_EnOrDi
 *
 * @brief		- This function enables or disables the capture/compare register
 * 			  	  for the given timer channel
 *
 * @param[in]	- Base address of the TIM peripheral
 * @param[in]	- Macros: Enable or Disable
 *  @param[in]	- Channel Number
 *
 *
 * @return		- None
 *
 * @Note		- here we configure the CCxE bits of CCER register to enable or
 * 				  disable the capture/ compare register of the timer as when this register is enable
 * 				  then only the value of CCRx is compared with the CNT (counter value)
 * *
 *****************************************************************/
void TIM_OC_EnOrDi(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber , uint8_t EnOrDi)
{
	if(ChannelNumber == TIM_IC_Channel_1)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC1E);
		}

	}else if(ChannelNumber == TIM_IC_Channel_2)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC2E);
		}


	}else if(ChannelNumber == TIM_IC_Channel_3)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC3E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC3E);
		}


	}else if(ChannelNumber == TIM_IC_Channel_4)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC4E);
		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC3E);
		}


	}

}
/*****************************************************************
 * @fn			- TIM_OC_EnOrDi_WithIT
 *
 * @brief		- This function enables or disables the capture/compare register
 * 			  	  for the given timer channel with interrupt so whenever CCRx value mataches counter
 * 			  	  value a interrupt is generated
 *
 * @param[in]	- Base address of the TIM peripheral
 * @param[in]	- Macros: Enable or Disable
 *  @param[in]	- Channel Number
 *
 *
 * @return		- None
 *
 * @Note		- here we configure the CCxE bits of CCER register to enable or
 * 				  disable the capture/ compare register of the timer as when this register is enable
 * 				  then only the value of CCRx is compared with the CNT (counter value)
 * 				- To enable the interrupt for the output capture channel CCxIE bit of CCER is configured
 * *
 *****************************************************************/

void TIM_OC_EnOrDi_WithIT(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber , uint8_t EnOrDi)
{
	if(ChannelNumber == TIM_IC_Channel_1)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC1IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC1E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC1IE);
		}

	}else if(ChannelNumber == TIM_IC_Channel_2)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC2IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC2E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC2IE);
		}


	}else if(ChannelNumber == TIM_IC_Channel_3)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC3E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC3IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC3E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC3IE);
		}


	}else if(ChannelNumber == TIM_IC_Channel_4)
	{
		if(EnOrDi == ENABLE)
		{
			pTIMx->CCER |=(ENABLE << TIMx_CCER_CC4E);
			pTIMx->DIER |=(ENABLE << TIMx_DIER_CC4IE);

		}else
		{
			pTIMx->CCER &= ~(DISABLE << TIMx_CCER_CC4E);
			pTIMx->DIER &= ~(DISABLE << TIMx_DIER_CC4IE);
		}


	}
}


/*
 * function to get PWM
 *
 */
/*****************************************************************
 * @fn			- TIM_PWM_Init
 *
 * @brief		- This function is to configure the timer's output channel to produce PWM
 *
 *
 * @param[in]	- Timer Handle structure
 * @param[in]	- Timer output campare structure
 *
 * @return		- None
 *
 * @Note
 * 				- In this we configure the selected channel of the timer to produce
 * 				  PWM as the output, PWM is produce by comparing the CNT and CCRx value
 * 				  their are 2 counting modes of PWM which decide the PWM output according to comparison
 * 				  both the modes of the PWM can can edge aligned or center aligned,
 * 				-in this function we configure the
 * 				 - PWM mode
 * 				 - PWM duty clycle
 * 				 - PWM Period
 * 				 - enable the output preload register so we can get a continos PWM output
 * 				 -  set the pin polarity untill PWM in not started

 *
 *****************************************************************/


void TIM_PWM_Init(TIM_Handle_t *pTimHandle, TIM_OC_InitTypeDef *pConfigOC)
{
	uint32_t ChannelName = pTimHandle->TIM_ChannelNumber;
	switch(ChannelName)
	{
		case TIM_OC_Channel_1:
		{
			//1.Configure the output pin:

			//A.make channel in output mode
			pTimHandle->pTIMx->CCMR1 &= ~( 1 << TIMx_CCMR1_CC1S);

			//B.Selecting the polarity
			pTimHandle->pTIMx->CCER &= ~(To_clear_onebit << TIMx_CCER_CC1P);
			pTimHandle->pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC1P);

			//2.Select the PWM mode
			if( (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode1 ) || (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode2) )
			{
				pTimHandle->pTIMx->CCMR1 &= ~(7 << TIMx_CCMR1_OC1M);
				pTimHandle->pTIMx->CCMR1 |=  (pConfigOC->OC_Mode << TIMx_CCMR1_OC1M);
			}
			else
			{
				void TIMx_Error();
			}

			//3.Programming  the period and the duty cycle
			pTimHandle->pTIMx->ARR  = (pTimHandle->TIMConfig.Period);
			pTimHandle->pTIMx->CCR1 = (pConfigOC->Pulse);

			//4. a) Setting the preload bit
			pTimHandle->pTIMx->CCMR1 |= (ENABLE <<TIMx_CCMR1_OC1PE);

			//b)Set  ARPE bit in the CR1 register
			pTimHandle->pTIMx->CR1 |= (ENABLE <<TIMx_CR1_ARPE);

		      /* Configure the Output Fast mode */
			pTimHandle->pTIMx->CCMR1 &= ~( pConfigOC->OC_FastMode << TIMx_CCMR1_OC1FE);
			pTimHandle->pTIMx->CCMR1 |= ( pConfigOC->OC_FastMode << TIMx_CCMR1_OC1FE);


			//5.Set the counting mode (from @TIM_Counter_Mode)
			//here if is used as we have different bit for center aligned and up/down
			 if( (pTimHandle->TIMConfig.CounterMode == (TIM_COUNTERMODE_UP ||  TIM_COUNTERMODE_DOWN ) ) )
			 {
				 pTimHandle->pTIMx->CR1 &= ~(1 << TIMx_CR1_DIR );
				 pTimHandle->pTIMx->CR1 |=  (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_DIR );
			 }else
			 {
				 pTimHandle->pTIMx->CR1 &= ~(3 << TIMx_CR1_CMS );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_CMS );
			 }

			 //6.Enabling the capture compare
			 pTimHandle->pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);



			 //7.now we can enable the counter by setting the CEN bit CR1 register

			break;
		}

		case TIM_OC_Channel_2:
		{
			pTimHandle->pTIMx->CCMR1 &= ~( 1 << TIMx_CCMR1_CC2S);

			pTimHandle->pTIMx->CCER &= ~(To_clear_onebit << TIMx_CCER_CC2P);
			pTimHandle->pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC2P);

			if( (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode1 ) || (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode2) )
			{
				pTimHandle->pTIMx->CCMR1 &= ~(7 << TIMx_CCMR1_OC2M);
				pTimHandle->pTIMx->CCMR1 |= (pConfigOC->OC_Mode << TIMx_CCMR1_OC2M);
			}
			else
			{
				void TIMx_Error();
			}

			pTimHandle->pTIMx->ARR = (pTimHandle->TIMConfig.Period);
			pTimHandle->pTIMx->CCR2 = (pConfigOC->Pulse);

			pTimHandle->pTIMx->CCMR1 |= (ENABLE <<TIMx_CCMR1_OC2PE);
			pTimHandle->pTIMx->CR1 |= (ENABLE <<TIMx_CR1_ARPE);

		      /* Configure the Output Fast mode */
			pTimHandle->pTIMx->CCMR1 &= ~( pConfigOC->OC_FastMode << TIMx_CCMR1_OC2FE);
			pTimHandle->pTIMx->CCMR1 |= ( pConfigOC->OC_FastMode << TIMx_CCMR1_OC2FE);


			if( (pTimHandle->TIMConfig.CounterMode == (TIM_COUNTERMODE_UP ||  TIM_COUNTERMODE_DOWN ) ) )
			{
				 pTimHandle->pTIMx->CR1 &= ~(1 << TIMx_CR1_DIR );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_DIR );
			}else
			{
				 pTimHandle->pTIMx->CR1 &= ~(3 << TIMx_CR1_CMS );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_CMS );
			}

			pTimHandle->pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);

			//now we can enable the counter by setting the CEN bit CR1 reg




			break;
		}

		case TIM_OC_Channel_3:
		{
			pTimHandle->pTIMx->CCMR2 &= ~( 1 << TIMx_CCMR2_CC3S);

			pTimHandle->pTIMx->CCER &= ~(1 << TIMx_CCER_CC3P);
			pTimHandle->pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC3P);

			if( (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode1 ) || (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode2) )
			{
				pTimHandle->pTIMx->CCMR2 &= ~(7 << TIMx_CCMR2_OC3M);
				pTimHandle->pTIMx->CCMR2 |= (pConfigOC->OC_Mode << TIMx_CCMR2_OC3M);
			}
			else
			{
				void TIMx_Error();
			}

			pTimHandle->pTIMx->ARR = (pTimHandle->TIMConfig.Period);
			pTimHandle->pTIMx->CCR3 = (pConfigOC->Pulse);

			pTimHandle->pTIMx->CCMR2 |= (ENABLE <<TIMx_CCMR2_OC3PE);
			pTimHandle->pTIMx->CR1 |= (ENABLE <<TIMx_CR1_ARPE);

		      /* Configure the Output Fast mode */
			pTimHandle->pTIMx->CCMR2 &= ~( pConfigOC->OC_FastMode << TIMx_CCMR2_OC3FE);
			pTimHandle->pTIMx->CCMR2 |= ( pConfigOC->OC_FastMode << TIMx_CCMR2_OC3FE);


			if( (pTimHandle->TIMConfig.CounterMode == (TIM_COUNTERMODE_UP ||  TIM_COUNTERMODE_DOWN ) ) )
			{
				 pTimHandle->pTIMx->CR1 &= ~(1 << TIMx_CR1_DIR );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_DIR );
			}else
			{
				 pTimHandle->pTIMx->CR1 &= ~(3 << TIMx_CR1_CMS );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_CMS );
			}

			pTimHandle->pTIMx->CCER |=(ENABLE << TIMx_CCER_CC3E);

			//now we can enable the counter by setting the CEN bit CR1 reg
			break;
		}

		case TIM_OC_Channel_4:
		{
			pTimHandle->pTIMx->CCMR2 &= ~( 1 << TIMx_CCMR2_CC4S);

			pTimHandle->pTIMx->CCER &= ~(1 << TIMx_CCER_CC4P);
			pTimHandle->pTIMx->CCER |= (pConfigOC->OC_Polarity << TIMx_CCER_CC4P);

			if( (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode1 ) || (pConfigOC->OC_Mode == TIM_OC_MODE_PWM_Mode2) )
			{
				pTimHandle->pTIMx->CCMR2 &= ~(7 << TIMx_CCMR2_OC4M);
				pTimHandle->pTIMx->CCMR2 |= (pConfigOC->OC_Mode << TIMx_CCMR2_OC4M);
			}
			else
			{
				void TIMx_Error();
			}

			pTimHandle->pTIMx->ARR = (pTimHandle->TIMConfig.Period);
			pTimHandle->pTIMx->CCR2 = (pConfigOC->Pulse);

			pTimHandle->pTIMx->CCMR2 |= (ENABLE <<TIMx_CCMR2_OC4PE);
			pTimHandle->pTIMx->CR1 |= (ENABLE <<TIMx_CR1_ARPE);

			pTimHandle->pTIMx->CCMR2 &= ~( pConfigOC->OC_FastMode << TIMx_CCMR2_OC4FE);
			pTimHandle->pTIMx->CCMR2 |= ( pConfigOC->OC_FastMode << TIMx_CCMR2_OC4FE);


			if( (pTimHandle->TIMConfig.CounterMode == (TIM_COUNTERMODE_UP ||  TIM_COUNTERMODE_DOWN ) ) )
			{
				 pTimHandle->pTIMx->CR1 &= ~(1 << TIMx_CR1_DIR );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_DIR );
			}else
			{
				 pTimHandle->pTIMx->CR1 &= ~(3 << TIMx_CR1_CMS );
				 pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig.CounterMode << TIMx_CR1_CMS );
			}

			pTimHandle->pTIMx->CCER |=(ENABLE << TIMx_CCER_CC4E);

			//now we can enable the counter by setting the CEN bit CR1 reg
			break;
		}



	}

}
void PWM_Start(TIM_RegDef_t	*pTIMx)
{
	pTIMx->EGR |=(1<<TIMx_EGR_UG);
	pTIMx->CR1 |=(1<<TIMx_CR1_CEN);
}


/* One pulse mode is :
 * To control an output waveform, or to indicate when a period of time has elaps
 */
/*****************************************************************
 * @fn			- TIM_OnePulseMode_Init
 *
 * @brief		- this function is to configure the output capture mode of the timer
 *
 * @param[in]	- Timer base address
 *  @param[in]	- Timer one pulse config structure
 *
 *
 * @return		- NONE
 *
 * @Note		- before calling this function  select the clock  and configure the ARR and CCRx registers
 *
 * 				- This function is to configure the timer channel as in output compare mode
 * 				  i.e we want the timer pin to produce output when CNT and CCRx value matches
 * 				  as when they match the channel output value changes according to the output compare mode
 * 				  selected  by the user
 *
 * 				- after this set the CNT bit of TIMx_CR1 to start the counter
 * 				-configure the CCxIE BIT OR CCxDE BIT  if we want interrupt or DMA request
 * 				 or else we can always go for polling by constnly checking the flag register
 *
 *
 *****************************************************************/

void TIM_OnePulseMode_Init(TIMx_OnePulse_Init_t *pOPMode ,TIM_Handle_t *pTimHandle)
{
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;
	uint32_t temp3 = 0;

	temp1 = pTimHandle->pTIMx->CCER;
	temp2 = pTimHandle->pTIMx->CCMR1;
	temp3 = pTimHandle->pTIMx->SMCR;


	switch(pOPMode->InputChannel)
	{
		case TIM_IC_Channel_1:

			temp1 &= ~(1  << TIMx_CCER_CC1E );

			//configure the channel to be mapped on TI1, i.e
			//input of channel 1 will be compared
			temp2 &= ~(3 << TIMx_CCMR1_CC1S);
			temp2 |= (pOPMode->ICSlection << TIMx_CCMR1_CC1S);

			temp2 &= ~(15 << TIMx_CCMR1_IC1F);
			temp2 |= (pOPMode->ICFilter <<TIMx_CCMR1_IC1F);

			//temp1 &=~ ( (1 << TIMx_CCER_CC1P) & ( 1 << TIMx_CCER_CC1NP) );
			//temp1 |= (pOPMode->ICPolarity << TIMx_CCER_CC1P )
			//temp2

			//setting the i/p capture polarity
			temp1 |= (pOPMode->ICPolarity <<  TIMx_CCER_CC1P);

			temp2 &= ~(3 << TIMx_CCMR1_IC1PSC);

			//setting the I/p trigger for the slave mode , which will be used for the counter clock
			temp3 &= ~(7 << TIMx_SMCR_TS);
			temp3 |= (TIM_CLOCKSOURCE_EXTMODE1_TI1FP1 << TIMx_SMCR_TS);

			//setting trigger mode as the slave mode
			temp3 &= ~(7 << TIMx_SMCR_SMS);
			temp3 |= (6<< TIMx_SMCR_SMS);


			break;

		case TIM_IC_Channel_2:

			temp1 &= ~(1  << TIMx_CCER_CC2E );

			temp2 &= ~(3 << TIMx_CCMR1_CC2S);
			temp2 |= (pOPMode->ICSlection << TIMx_CCMR1_CC2S);

			temp2 &= ~(15 << TIMx_CCMR1_IC2F);
			temp2 |= (pOPMode->ICFilter <<TIMx_CCMR1_IC2F);

			//temp1 &=~ ( (1 << TIMx_CCER_CC1P) & ( 1 << TIMx_CCER_CC1NP) );
			//temp1 |= (pOPMode->ICPolarity << TIMx_CCER_CC1P )
			//temp2
			temp1 |= (pOPMode->ICPolarity <<  TIMx_CCER_CC2P);

			temp2 &= ~(3 << TIMx_CCMR1_IC2PSC);

			temp3 &= ~(7 << TIMx_SMCR_TS);
			temp3 |= (TIM_CLOCKSOURCE_EXTMODE1_TI2FP2 << TIMx_SMCR_TS);

			temp3 &= ~(7 << TIMx_SMCR_SMS);
			temp3 |= (6 << TIMx_SMCR_SMS);

			break;

	}
	switch(pOPMode->OutputChannel)
	{
		case TIM_IC_Channel_1:

		temp2 &= ~(1 << TIMx_CCMR1_OC1M);
		temp2 &= ~(1 << TIMx_CCMR1_CC1S);

		//setect the output compare mode, note it can be PWM1 or PWM2
		temp2 |= ( pOPMode->OCMode << TIMx_CCMR1_OC1M);
		//set the output mode polarity
		temp1 &= ~(1 << TIMx_CCER_CC1P);
		temp1 |= (pOPMode->OCPolarity << TIMx_CCER_CC1P);
		 pTimHandle->pTIMx->ARR =pOPMode->Pulse; //CONFUSION BETWEEN cr1 OR ARR

		break;


		case TIM_IC_Channel_2:

			temp2 &= ~(1 << TIMx_CCMR1_OC2M);
			temp2 &= ~(1 << TIMx_CCMR1_CC2S);

			temp2 |= ( pOPMode->OCMode << TIMx_CCMR1_OC2M);
			temp1 &= ~(1 << TIMx_CCER_CC2P);
			temp1 |= (pOPMode->OCPolarity << TIMx_CCER_CC2P);
			 pTimHandle->pTIMx->ARR =pOPMode->Pulse;

			break;

	}

	 pTimHandle->pTIMx->CCER = temp1;
	 pTimHandle->pTIMx->CCMR1 = temp2;
	 pTimHandle->pTIMx->SMCR = temp3;

	// pTimHandle->pTIMx->CR1 &= ~(pTimHandle->TIMConfig->CounterMode << TIMx_CR1_DIR);
	// pTimHandle->pTIMx->CR1 |= (pTimHandle->TIMConfig->CounterMode << TIMx_CR1_DIR);

	 pTimHandle->pTIMx->CR1 &= ~(1<< TIMx_CR1_CMS);

	 pTimHandle->pTIMx->CCMR1 |= (pOPMode->OCMode << TIMx_CCMR1_OC1M);

	 pTimHandle->pTIMx->CR1 |= (pOPMode->OPMode << TIMx_CR1_OPM);



}

void TIM_SlaveController_Int(TIM_RegDef_t	*pTIMx ,TIM_SlaveConfigTypeDef *pSlaveConfig)
{
	//1.Select the slavecontroller mode
	if(!(pSlaveConfig->SlaveMode == TIM_SLAVEMODE_RESET || pSlaveConfig->SlaveMode == TIM_SLAVEMODE_GATED ||
		pSlaveConfig->SlaveMode == TIM_SLAVEMODE_TRIGGER ||pSlaveConfig->SlaveMode == TIM_SLAVEMODE_EXTERNAL1))
	{
		TIMx_Error();
	}
	pTIMx->SMCR &= ~(7 << TIMx_SMCR_SMS);
	pTIMx->SMCR |= (pSlaveConfig->SlaveMode << TIMx_SMCR_SMS);

	//2.Config the polarity
	if(pSlaveConfig->InputTrigger == TIM_TRIG_SELECTION_TI1FP1)
	{
		//2.Config the polarity
		pTIMx->CCER  &= ~(1 << TIMx_CCER_CC1P);
		pTIMx->CCER  &= ~(1 << TIMx_CCER_CC1NP);
		pTIMx->CCER  |= (pSlaveConfig->TriggerPolarity << TIMx_CCER_CC1P);

		//3.Config the filter
		pTIMx->CCMR1 |= (pSlaveConfig->TriggerFilter<<TIMx_CCMR1_IC1F);
		pTIMx->CCMR1 |=(1 << TIMx_CCMR1_CC1S);


	}else if (pSlaveConfig->InputTrigger == TIM_TRIG_SELECTION_TI2FP2)
	{
		//2.Config the polarity
		pTIMx->CCER &= ~(1 << TIMx_CCER_CC2P);
		pTIMx->CCER &= ~(1 << TIMx_CCER_CC2NP);
		pTIMx->CCER |= 	(pSlaveConfig->TriggerPolarity << TIMx_CCER_CC2P);
		//3.Config the filter
		pTIMx->CCMR1 |= (pSlaveConfig->TriggerFilter<<TIMx_CCMR1_IC2F);
		pTIMx->CCMR1 |=(1 << TIMx_CCMR1_CC2S);


	}



	//4.select the input source
	pTIMx->SMCR &= ~(7 << TIMx_SMCR_TS);
	pTIMx->SMCR |=(pSlaveConfig->InputTrigger << TIMx_SMCR_TS);

	//5. ENABLE THE COUNTER
}
/*****************************************************************
 * @fn			- TIM_IRQInterruptConfig
 *
 * @brief		- This function is to enable the interrupt  for  the TIM
 * 				  in the NVIC , as timer produce many interrupts , like
 * 				  -overflow
 * 				  -underflow
 * 				  -input capture complete
 * 				  -output capture
 * 				  -etc.
 *
 * @param[in]	- IRQ number
 * @param[in]	- EnOrDi
 *
 *
 * @return		- None
 *
 * @Note		- this function is to enable the timer interrupt , that is when the timer
 * 				 - that is when ever an update eb=vent occurs the timer produces an interrupt ,
 * 				 if we did't enable the interrupt we have to keep poling the event flags of the required task
 *
 *****************************************************************/
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/*****************************************************************
 * @fn			- TIM_IRQPriorityConfig
 *
 * @brief		- This function is to set the priority of  the interrupt  for the TIM
 * 				  peripheral
 *
 * @param[in]	- IRQ number
 * @param[in]	- IRQ priority
 *
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void TIM_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
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
 * @fn			- TIM_BASIC_IRQ_HANDLER
 *
 * @brief		- This function is called from the interrupt handler
 *
 * @param[in]	- Timer base address
 * @param[in]	- Timer flag name
 *
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/

void TIM_BASIC_IRQ_HANDLER(TIM_RegDef_t *pTIMx)
{
	if(pTIMx->SR & (1 << 0))
	{
		pTIMx->SR = 0 ;
	}
}


void TIM_ClearFlag(TIM_RegDef_t *pTIMx, uint16_t StatusFlagName)
{
	pTIMx->SR &= ~( StatusFlagName);
}






void TIMx_Error()
{

}




void TIM_OnePulse_Start(TIM_RegDef_t *pTIMx , uint32_t delayValue  )
{
	pTIMx->CCR2 = delayValue;

	pTIMx->CR1  = (1 << TIMx_CR1_OPM);
}

/*
 * function to configure the clock source
 */

/*****************************************************************
 * @fn			- TIM_ClockConfig
 *
 * @brief		- This function is to configure the clock source for the timer
 * 				- it is used for basic timer
 *
 * @param[in]	- Timer base address
 * @param[in]	- Timer clock config structure
 *
 * @return		- None
 *
 * @Note		- here we select the clock source for the timer, that we want to use to trigger
 * 				  the counter of the timer , their are various options to select from
 * 				  -external mode 1 ( in this we take clock from TI1 or TI2)
 * 				  -external mode 2 (in this we take clock from the EXT pin of the timer)
 * 				  -use internal clock (i.e the clock provided by the RCC)
 * 				  -use other timers as the clock source
 *
 * 				- if using external clock , then we have to configure the required GPIO
 * 				  pin to act as the required alternate function to take clock externally
 *
 * 				- : It is possible to simultaneously use external clock mode 2 with the following slave modes:
					reset mode, gated mode and trigger mode. Nevertheless, TRGI must not be connected to
					ETRF in this case (TS bits must not be 111),i.e we have to use TIM_CLOCKSOURCE_ETRMODE2
					we configure the external mode 2 , TIM_CLOCKSOURCE_EXTMODE2_ETRF can't be used

 *
 *****************************************************************/

void TIM_ClockConfig(TIM_RegDef_t *pTIMx ,TIM_ClockConfigTypeDef *pClockConfig )
{
	switch(pClockConfig->ClockSource )
	{
		case TIM_CLOCKSOURCE_INTERNAL:
		{
			//TO select internal clock for the timer we have to disable the slave mode
			//of the timer it is done by configuring SMS bit of TIM_SMCR to 00
			pTIMx->SMCR &= ~( 1 << TIMx_SMCR_SMS);
			break;

		}
		case TIM_CLOCKSOURCE_ETRMODE2:
		{


			//1.configure the  External trigger filter
			pTIMx->SMCR &= ~(1 << TIMx_SMCR_ETF);
			pTIMx->SMCR |= (pClockConfig->ClockFilter << TIMx_SMCR_ETF);

			//2.configure the External trigger prescaler
			pTIMx->SMCR &= ~(3 << TIMx_SMCR_ETPS);
			pTIMx->SMCR |= (pClockConfig->ClockPrescaler << TIMx_SMCR_ETPS);

			//3.configure the  External trigger polarity
			pTIMx->SMCR &= ~(1 << TIMx_SMCR_ETP);
			pTIMx->SMCR |= (pClockConfig->ClockPolarity << TIMx_SMCR_ETP);

			//4. External clock enable
			pTIMx->SMCR |= (ENABLE << TIMx_SMCR_ECE);
			break;

		}
		case TIM_CLOCKSOURCE_EXTMODE2_ETRF:
		{

			//1.configure the  External trigger filter
			pTIMx->SMCR &= ~(1 << TIMx_SMCR_ETF);
			pTIMx->SMCR |= (pClockConfig->ClockFilter << TIMx_SMCR_ETF);

			//2.configure the External trigger prescaler
			pTIMx->SMCR &= ~(3 << TIMx_SMCR_ETPS);
			pTIMx->SMCR |= (pClockConfig->ClockPrescaler << TIMx_SMCR_ETPS);

			//3.configure the  External trigger polarity
			pTIMx->SMCR &= ~(1 << TIMx_SMCR_ETP);
			pTIMx->SMCR |= (pClockConfig->ClockPolarity << TIMx_SMCR_ETP);

			//4.configure the TS mux to give ETRX as output
			pTIMx->SMCR |= ( TIM_CLOCKSOURCE_EXTMODE2_ETRF << TIMx_SMCR_TS);
			//5. configure the slave controller to take TRIG as input
			// as TRIG is conifgured to provide ETRF So, like this we can ETRF single by this config
			pTIMx->SMCR |= ( 7 << TIMx_SMCR_SMS);


			break;
		}
		case TIM_CLOCKSOURCE_EXTMODE1_TI1FP1:
		{
			//NOTE = Even though any value written to the capture/compare selection (CCxS )
			//control bit-field except the 00 value sets the timer channel in input mode,
			//the proffered right value to configure is CCxS = 01.

			//HERE WE USE CHANNEL 1 AS CLOCK SOURCE

			//1.to config TI1 as input and TI1FP1 as the signal
			pTIMx->CCMR1 &= ~( 3 << TIMx_CCMR1_CC1S); //clear the bit field
			pTIMx->CCMR1 |= ( 1 << TIMx_CCMR1_CC1S); //configure the bit field

			//2. to configure the input capture trigger polarity
			pTIMx->CCER	&= ~(1 << TIMx_CCER_CC1P);
			pTIMx->CCER	&= ~(1 << TIMx_CCER_CC1NP);
			if(pClockConfig->ClockPolarity == TIM_CLOCKPOLARITY_BOTHEDGE)
			{
				pTIMx->CCER	|= (1 << TIMx_CCER_CC1P);
				pTIMx->CCER	|= (1 << TIMx_CCER_CC1NP);

			}else
			{

				pTIMx->CCER	|= (pClockConfig->ClockPolarity << TIMx_CCER_CC1P);

			}

			//3.to configure the filter settings
			pTIMx->CCMR1 &= ~( 15 << TIMx_CCMR1_IC1F);
			pTIMx->CCMR1 |= ( pClockConfig->ClockFilter<< TIMx_CCMR1_IC1F);
			//4.configure the pre scalar of input(it is of no use in this mode)
			pTIMx->CCMR1 |= (3 << TIMx_CCMR1_IC1PSC);
			pTIMx->CCMR1 |= (pClockConfig->ClockPrescaler << TIMx_CCMR1_IC1PSC);

			//5.configure the clock input source to the clock controller
			pTIMx->SMCR |= (7 << TIMx_SMCR_TS);
			pTIMx->SMCR |= (pClockConfig->ClockSource << TIMx_SMCR_TS);
			//6.set the timer clock mode as external clock mode
			pTIMx->SMCR |= ( 7 << TIMx_SMCR_SMS);



		}
		case TIM_CLOCKSOURCE_EXTMODE1_TI2FP2:
		{
			//NOTE = Even though any value written to the capture/compare selection (CCxS )
			//control bit-field except the 00 value sets the timer channel in input mode,
			//the right value to configure is CCxS = 01.

			//SIMILAR TO TIM_CLOCKSOURCE_EXTMODE1_TI1FP1
			//HERE WE USE CHANNEL 2 AS CLOCK SOURCE

			pTIMx->CCMR1 &= ~( 3 << TIMx_CCMR1_CC2S);
			pTIMx->CCMR1 |= ( 1 << TIMx_CCMR1_CC2S);

			pTIMx->CCER	 &= ~(pClockConfig->ClockPolarity << TIMx_CCER_CC2P);
			pTIMx->CCER	 &= ~(pClockConfig->ClockPolarity << TIMx_CCER_CC2NP);
			if(pClockConfig->ClockPolarity == TIM_CLOCKPOLARITY_BOTHEDGE)
			{
				pTIMx->CCER	|= (1 << TIMx_CCER_CC2P);
				pTIMx->CCER	|= (1 << TIMx_CCER_CC2NP);

			}else
			{

				pTIMx->CCER	|= (pClockConfig->ClockPolarity << TIMx_CCER_CC2P);

			}


			pTIMx->CCMR1 &= ~(3 << TIMx_CCMR1_IC2PSC);
			pTIMx->CCMR1 |= (pClockConfig->ClockPrescaler << TIMx_CCMR1_IC2PSC);

			//Configure the input filter
			pTIMx->CCMR1 &= ~( 15 << TIMx_CCMR1_IC1F);
			pTIMx->CCMR1 |= ( pClockConfig->ClockFilter	<< TIMx_CCMR1_IC1F);

			//select the external clock sourse
			pTIMx->SMCR  &= ~(7<< TIMx_SMCR_TS);
			pTIMx->SMCR  |= (pClockConfig->ClockSource 	<< TIMx_SMCR_TS);

			//configure the external clock mode
			pTIMx->SMCR  &= ~( 7 << TIMx_SMCR_SMS);
			pTIMx->SMCR  |= ( 7 << TIMx_SMCR_SMS);


		}



		//NOW SET CEN control bit-field of CR1 to enable the counter for this we use
		//void TIM_BASIC_ENABLE(TIM_RegDef_t *pTIMx)


	}
}
/*

****************************************************************
 * @fn			- TIM_IC_SpecialMode
 *
 * @brief		- this function is the particular case of input capture
 *
 * @param[in]	- Timer base address
 *  @param[in]	- Timer input capture structure
 * @param[in]	- Timer clock config structure
 *
 *
 * @return		- OK or ERROR
 *
 * @Note		-  this function is the particular case of input capture in which
 * 				   we use external mode 1 of clock , we have to use TIxFPx clock
 * 				   in this after every rising edge the timer resets and before reseting
 * 				   it gives us the count value by which we can calculate the reset value
 * 				   can be only used for channel 1 and channel 2
 *

 *
 ****************************************************************

uint32_t TIM_IC_SpecialMode(TIM_RegDef_t*pTIMx , TIM_IC_Init_Handle_t *pTimxHandle , TIM_ClockConfigTypeDef *pClockConfig )
{
	//checking the givin parameters
	if( !(pClockConfig->ClockSource == (TIM_CLOCKSOURCE_EXTMODE1_TI1FP1 || TIM_CLOCKSOURCE_EXTMODE1_TI2FP2) )
			&&  (pTimxHandle->TIM_ChannelNumber == (TIM_IC_Channel_1 || TIM_IC_Channel_2 )) )
	{
		return ERROR;
	}
	//configure the clock source
	pTIMx->SMCR |= (pClockConfig->ClockSource << TIMx_SMCR_TS);
	pTIMx->SMCR |= ( 4 << TIMx_SMCR_SMS);

	if (pClockConfig->ClockSource == TIM_CLOCKSOURCE_EXTMODE1_TI1FP1 )
	{


	//setting the mode
	pTIMx->CCMR1 |= (pTimxHandle->ICSelection << TIMx_CCMR1_CC1S);
	pTIMx->CCMR1 |= (pTimxHandle->IC_filter << TIMx_CCMR1_IC1F);
	pTIMx->CCER |= (pTimxHandle->ICPolarity << TIMx_CCER_CC1P);
	pTIMx->CCMR1 |= (pTimxHandle->ICPrescaler << TIMx_CCMR1_IC1PSC);
	//capture of the counter value into the input capture/compare register
	pTIMx->CCER |=(ENABLE << TIMx_CCER_CC1E);
	}else
	{
		pTIMx->CCMR1 |= (pTimxHandle->IC_filter << TIMx_CCMR1_IC2F);
		pTIMx->CCMR1 |= (pTimxHandle->ICSelection << TIMx_CCMR1_CC2S);
		pTIMx->CCER |= (pTimxHandle->ICPolarity << TIMx_CCER_CC2P);
		pTIMx->CCMR1 |= (pTimxHandle->ICPrescaler << TIMx_CCMR1_IC2PSC);
		//capture of the counter value into the input capture/compare register
		pTIMx->CCER |=(ENABLE << TIMx_CCER_CC2E);

	}

	return OK;
}
*/
