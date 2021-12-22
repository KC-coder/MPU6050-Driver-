/*
 * stm32F407xx_RCC_driver.c
 *
 *  Created on: 17-Sep-2020
 *      Author: KAMAL CHOPRA
 */

#include "stm32f407xx.h"
#include "stm32F407xx_RCC_driver.h"





/*
 * using the clk tree
 * 1)find the source of the clock (i.e HSE)
 * 2)check what is the value of the system prescaler
 * 3)check the value of prescaler of   APBx bus on which it is connected
 * 4)then we get the value of clk to the I2C
 */
uint32_t RCC__GetPLLOutputClock()
{
	return 0;

}
uint16_t AHB_PreScaler [8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScalar[4] = {2,4,8,16};

//this is a generic function to calculate the value of pclk1 i.e APB1 clk
uint32_t RCC_getPCLK1Value(void)
{
	//in RCC_CFGR register,bit SW0 and SW1 to get system clock status
	uint8_t  temp , ahbp ,apb1p;
	uint32_t clksrc;
	uint32_t  SystemClk ;
	uint32_t pclk1;
	//here bring bits 2and 3 to 0th and 1st position , by right shifing the 2 bits then masking other bits
	//using ' & 'then storing the value we got in the variable

	clksrc = (RCC->CFGR >>2) & 0x3;

	if(clksrc == 0)
	{
		//sclk sourse is HSI which is 16MHz
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		//sclk source is HSE which is 8MHZ
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		//clksrc source is Pll
		SystemClk = RCC__GetPLLOutputClock();
	}

	//finding the value of AHB prescaler

	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp <8)
	{
		//as for values less than 8 prescaller is 1
		ahbp = 1;
	}else
	{
		//doing  -8 as first 8 values of temp are not included in the arry
		//they are in the previous function
		ahbp = AHB_PreScaler[temp - 8];
	}


	//finding values of APB1 prescaler
	//for the we have to refer RCC_CFGR 's 10th bit field

	temp = (RCC->CFGR >> 10) & 0x7;

	if(temp < 4)
	{
		//as for values less than 4 prescaller is 1
		apb1p = 1;
	}else
	{
		//doing  -4 as first 4 values of temp are not included in the arry
		//they are in the previous function
		apb1p =APB1_PreScalar[temp - 4];
	}

	pclk1 = ( (SystemClk / ahbp ) / apb1p );

	return pclk1;

}


uint32_t RCC_getPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScalar[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}
void RCC_OsclatorConfig(RCC_SYSClkType_Select_t *pOsclator )
{


	//1.setting the selected clk
	if( pOsclator->ClkType_Select == RCC_OSCILLATORTYPE_HSE )
	{
		if( (pOsclator->HSE_State != HSE_CLOCK_BYPASSED ))
		{
			RCC->CR |= (pOsclator->HSE_State << RCC_CR_HSEON );

		}else
		{
			RCC->CR |= (pOsclator->HSE_State << RCC_CR_HSEBYP );
		}


	}else if( pOsclator->ClkType_Select == RCC_OSCILLATORTYPE_HSI )
	{
		RCC->CR |= (pOsclator->HSI_State << RCC_CR_HSION);

	}else if (pOsclator->ClkType_Select == RCC_OSCILLATORTYPE_PLL )
	{
		//write for the PLL
	}

	//2.selecting the clock to be used as system clk
	RCC->CFGR |= (pOsclator->ClkType_Select << RCC_CFGR_SW );


}

void RCC_ClkConfig(RCC_ClkInit_t *pCLKConfig)
{

	RCC->CFGR |= (pCLKConfig->AHBCLKDivider << RCC_CFGR_HPRE );

	RCC->CFGR |=(pCLKConfig->APB1CLKDivider << RCC_CFGR_PPRE1);

	RCC->CFGR |=(pCLKConfig->APB2CLKDivider << RCC_CFGR_PPRE2);


}



void RCC_MOC_Config(RCC_MCOx_Config_t *pMCOx )
{
	//For MCO1 configuring
	RCC->CFGR |= ( pMCOx->MCO1_CLK_TYPE  << RCC_CFGR_MCO1 );
	RCC->CFGR |= ( pMCOx->MCO1_CLK_Divider << RCC_CFGR_MCO1PRE );

	//For MCO2 configuring
	RCC->CFGR |= ( pMCOx->MCO2_CLK_TYPE << RCC_CFGR_MCO2);

	RCC->CFGR |= (pMCOx->MCO2_CLK_Divider << RCC_CFGR_MCO2PRE);

}


void RCC_RTC_Config(RCC_Hanale_t *pRTC)
{
		RCC->BDCR |= (pRTC->clockSourse << RCC_BDCR_RTCSEL);

		RCC->CFGR |= (pRTC->HSE_PerScalar << RCC_CFGR_RTCPRE);

}
