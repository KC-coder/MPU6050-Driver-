/*
 * stm32F407xx_ADC_driver.c
 *
 *  Created on: 07-Sep-2020
 *      Author: KAMAL CHOPRA
 */

#include"stm32f407xx.h"
#include"stm32F407xx_ADC_driver.h"
#include"stm32F407xx_gpio_driver.h"




void ADC_PeriClock(ADC_RegDef_t *pADC , uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		if(pADC == ADC1)
		{
			ADC1_PCLK_EN;
		}else if(pADC == ADC2)
		{
			ADC2_PCLK_EN;
		}else if(pADC == ADC3)
		{
			ADC3_PCLK_EN;
		}
	}else
	 {
		if(pADC == ADC1)
		{
			ADC1_PCLK_DN;
		}else if(pADC == ADC2)
		{
			ADC2_PCLK_DN;
		}else if(pADC == ADC3)
		{
			ADC3_PCLK_DN;
		}

	}
}


void ADC_Int(ADC_Handle_t *pADCHandle)
{
	uint32_t temp = 0;

	//configure resolution
	temp |= ( pADCHandle->ADC_Config.Resolution << ADC_CR1_RES);
	//configure overrun interrupt
	temp |=(pADCHandle->ADC_Config.overrun_IT << ADC_CR1_OVRIE);
	//EnorDi watchdog
	temp |=(pADCHandle->ADC_Config.Analog_watchdog_EnOrDi << ADC_CR1_AWDEN);
	//Discontinuous channel count
	temp|=(pADCHandle->ADC_Config.Discountionous_Channels << ADC_CR1_DISCNUM);
	//Discontinous mode on regular channels
	temp |=(pADCHandle->ADC_Config.Discontinuous_Mode_EnOrDi << ADC_CR1_DISCEN);

	pADCHandle->ADCx->CR1 |= temp;

	//=====================configuring CR2==================

	temp = 0;
	//configuring external trigger
	temp |= (pADCHandle->ADC_Config.Ext_trig_Type << ADC_CR2_EXTEN);
	//Configuring event that is used for external trigger
	temp|= (pADCHandle->ADC_Config.Ext_trig_Source << ADC_CR2_EXTSEL);
	//for continuous mode EnOrDis
	temp|= (pADCHandle->ADC_Config.Continous_Mode << ADC_CR2_CONT);

	pADCHandle->ADCx->CR2 |= temp;
	temp = 0;


}


void ADC_POWER_EnOrDi(ADC_RegDef_t *pADC , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pADC->CR2 |= (1<<ADC_CR2_ADON);

	}else
	{
		pADC->CR2 &= ~(1<<ADC_CR2_ADON);

	}


}
void ADC_Reg_Coversion_EnORDi(ADC_RegDef_t *pADC , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pADC->CR2 |= (1 << ADC_CR2_SWSTART);
	}else
	{
		pADC->CR2 &= ~(1 << ADC_CR2_SWSTART);
	}

}


void ADC_SCAN_MODE(ADC_RegDef_t *ADCx ,uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		ADCx->CR1 |= (ENABLE<<ADC_CR1_SCAN);
	}else
	{
		ADCx->CR1 &= ~(DISABLE<<ADC_CR1_SCAN);
	}
}


void ADC_CyclesPerSample (ADC_RegDef_t *pADC , uint16_t channel , uint8_t Cycles)
{
	uint16_t temp = 0;

	if(channel < 10)
	{
		pADC->SMPR2 |= (Cycles << (3*channel));
	}else
	{
		temp = (channel % 3 );
		pADC->SMPR1 |= (Cycles << (3*temp)) ;
	}
	temp = 0;
}

uint16_t ADC_ReadRegularData(ADC_RegDef_t *pADC)
{
	uint16_t data;
	data = (uint16_t)pADC->DR;
	return data;
}

uint16_t ADC_ReadInjectedData(ADC_RegDef_t *pADC , uint8_t Inj_channel)
{
	uint16_t data;
	data = (uint16_t)pADC->JDRx[Inj_channel];
	return data;
}

void ADC_CONFIG_WATCHDOG(ADC_Handle_t *pADC)
{
	pADC->ADCx->HTR = pADC->ADC_Config.Analog_watchdog_HT;
	pADC->ADCx->LTR = pADC->ADC_Config.Analog_watchdog_LT;
}

void ADC_Watchdog_CHSelect(ADC_RegDef_t *ADCx , uint16_t ADC_channel)
{
	ADCx->CR1 |=(ADC_channel << ADC_CR1_AWDCH);
}

void ADC_RegularChannelConfig(ADC_RegDef_t *pADC, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
	uint32_t tmpreg1 = 0, tmpreg2 = 0;

	if (ADC_Channel > ADC_CHANNEL_9)
	{
		/* Get the old register value */
	     tmpreg1 = pADC->SMPR1;
	     /* Calculate the mask to clear */
	     tmpreg2 = 7 << (3 * (ADC_Channel - 10));
	     /* Clear the old channel sample time */
	     tmpreg1 &= ~tmpreg2;
	     /* Calculate the mask to set */
	     tmpreg2 = (uint32_t)ADC_SampleTime << (3 * (ADC_Channel - 10));
	     /* Set the new channel sample time */
	     tmpreg1 |= tmpreg2;
	     /* Store the new register value */
	     pADC->SMPR1 = tmpreg1;
	 }else /* ADC_Channel include in ADC_Channel_[0..9] */
	 {
		 /* Get the old register value */
		 tmpreg1 = pADC->SMPR2;
		 /* Calculate the mask to clear */
	     tmpreg2 = 7 << (3 * ADC_Channel);
	     /* Clear the old channel sample time */
	     tmpreg1 &= ~tmpreg2;
	     /* Calculate the mask to set */
	     tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);
	    /* Set the new channel sample time */
	     tmpreg1 |= tmpreg2;
	     /* Store the new register value */
	     pADC->SMPR2 = tmpreg1;
	  }
		/* For Rank 1 to 6 */
	if (Rank < 7)
	{
		/* Get the old register value */
	     tmpreg1 = pADC->SQR3;
	     /* Calculate the mask to clear */
	     tmpreg2 = 0x1F << (5 * (Rank - 1));
	     /* Clear the old SQx bits for the selected rank */
	     tmpreg1 &= ~tmpreg2;
	     /* Calculate the mask to set */
	     tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 1));
	     /* Set the SQx bits for the selected rank */
	     tmpreg1 |= tmpreg2;
	     /* Store the new register value */
	     pADC->SQR3 = tmpreg1;
   }else if ((6 < Rank) &  (Rank < 13) )
   	  {
	     /* Get the old register value */
	     tmpreg1 = pADC->SQR2;
	     /* Calculate the mask to clear */
	     tmpreg2 = 0x1F << (5 * (Rank - 7));
	     /* Clear the old SQx bits for the selected rank */
	     tmpreg1 &= ~tmpreg2;
	     /* Calculate the mask to set */
	     tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 7));
	    /* Set the SQx bits for the selected rank */
	     tmpreg1 |= tmpreg2;
	     /* Store the new register value */
	     pADC->SQR2 = tmpreg1;
	   }else
	   {
	     /* Get the old register value */
	     tmpreg1 = pADC->SQR1;
	    /* Calculate the mask to clear */
	    tmpreg2 = 0x1F << (5 * (Rank - 13));
	    /* Clear the old SQx bits for the selected rank */
	    tmpreg1 &= ~tmpreg2;
	    /* Calculate the mask to set */
	    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 13));
	    /* Set the SQx bits for the selected rank */
	    tmpreg1 |= tmpreg2;
	     /* Store the new register value */
	    pADC->SQR1 = tmpreg1;
	  }
	}



