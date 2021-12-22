/*
 * stm32F407xx_RCC_driver.h
 *
 *  Created on: 17-Sep-2020
 *   Author: KAMAL CHOPRA
 *
 *  use - this header is used to select clock source and
 *  configure the clock of MCU
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include"stm32f407xx.h"

/*
 * using this structure we configure the selected clk the the desired value
 */

typedef struct
{
	uint32_t ClkType;			/*<possible value from @GPIO_PIN_MODES >*/

	uint32_t SYSCLK_Source;		/*<possible value from @GPIO_PIN_MODES >*/

	uint32_t AHBCLKDivider;		/*<possible value from @AHBCLKDivider >*/

	uint32_t APB1CLKDivider;	/*<possible value from @APB1CLKDivider >*/

	uint32_t APB2CLKDivider;	/*<possible value from @APB2CLKDivider >*/

}RCC_ClkInit_t;


/*
 * uisng this this structure we configure the pLL engine
 */

typedef struct
{

}RCC_PLL_EngineConfig_t;


/*
 * Using this structure  we select the osslicator from which we want to drive the clk
 */
typedef struct
{
	uint8_t ClkType_Select;

	uint8_t HSE_State;

	uint8_t HSI_State;

	uint8_t LSE_State;

	uint8_t LSI_State;

	uint8_t HSI_CalibrationValue;

	RCC_PLL_EngineConfig_t PLL;


}RCC_SYSClkType_Select_t;


/*used to output the MCU clock to some specific GPIO pins in alternate function
 * this is use full when we have to use or measure the MCU clk
 *
 * Here MCO - Microcontroller clock output
 */
typedef struct
{


	uint32_t MCO1_CLK_TYPE;

	uint32_t MCO2_CLK_TYPE;

	uint32_t MCO1_CLK_Divider;

	uint32_t MCO2_CLK_Divider;

}RCC_MCOx_Config_t;


/*
 * structure to configre the RTC peripheral clock
 */

typedef struct
{
	uint32_t clockSourse;

	uint32_t HSE_PerScalar;


}RCC_Hanale_t;





/*
 * @ClkType_Select
 */

#define RCC_OSCILLATORTYPE_HSI			0
#define RCC_OSCILLATORTYPE_HSE			1
#define RCC_OSCILLATORTYPE_PLL			2

/*
 * @HSE_State
 */

#define HSE_CLOCK_ON					1
#define HSE_CLOCK_OFF					0
#define HSE_CLOCK_BYPASSED				3



/*Used to divide the system clock for AHB bus
 * @AHBCLKDivider
 */

#define AHB_SCLK_DIV2					8
#define AHB_SCLK_DIV4					9
#define AHB_SCLK_DIV8					10
#define AHB_SCLK_DIV16					11
#define AHB_SCLK_DIV64					12
#define AHB_SCLK_DIV128					13
#define AHB_SCLK_DIV256					14
#define AHB_SCLK_DIV512					15

/*Used to divide the AHB bus clock for the APB1 bus
 * @APB1CLKDivider
 */

#define APB1_AHBCLK_DIV2				4
#define APB1_AHBCLK_DIV4				5
#define APB1_AHBCLK_DIV8				6
#define APB1_AHBCLK_DIV16				7

/*used to divide the AHB clock for the ABP2 bus
 * @APB2CLKDivider
 */

#define APB2_AHBCLK_DIV2				4
#define APB2_AHBCLK_DIV4				5
#define APB2_AHBCLK_DIV8				6
#define APB2_AHBCLK_DIV16				7


/*
 * used to select the oscillator type for the MCO1
 */

#define MCO1_TYPE_HSI					0
#define MCO1_TYPE_LSE					1
#define MCO1_TYPE_HSE					2
#define MCO1_TYPE_PLL					3

/*
 * used to select the oscillator type for the MCO1
 */

#define MCO2_TYPE_SYSCLK				0
#define MCO2_TYPE_PLLI2S				1
#define MCO2_TYPE_HSE					2
#define MCO2_TYPE_PLL					3


/*
 * Used to divide the clk of MCO1
 */

#define MCO1_CLK_DIV2					4
#define MCO1_CLK_DIV3					5
#define MCO1_CLK_DIV4					6
#define MCO1_CLK_DIV5					7


/*
 * Used to divide the clk of MCO2
 */

#define MCO2_CLK_DIV2					4
#define MCO2_CLK_DIV3					5
#define MCO2_CLK_DIV4					6
#define MCO2_CLK_DIV5					7

/*Macros to select the clock type for RTC
 * @RTC_clockSourse
 */


#define RTC_OSCILLATORTYPE_NOCLK	0
#define RTC_OSCILLATORTYPE_HSE		1
#define RTC_OSCILLATORTYPE_LSE		2
#define RTC_OSCILLATORTYPE_LSI		3

/*Macros for HSE division factor for RTC clock
 *
 */

#define RTC_HSE_CLOCK_DIV2			2
#define RTC_HSE_CLOCK_DIV3			3
#define RTC_HSE_CLOCK_DIV4			4
#define RTC_HSE_CLOCK_DIV5			5
#define RTC_HSE_CLOCK_DIV6			6
#define RTC_HSE_CLOCK_DIV7			7
#define RTC_HSE_CLOCK_DIV8			8
#define RTC_HSE_CLOCK_DIV9			9
#define RTC_HSE_CLOCK_DIV10			10
#define RTC_HSE_CLOCK_DIV11			11
#define RTC_HSE_CLOCK_DIV12			12
#define RTC_HSE_CLOCK_DIV13			13
#define RTC_HSE_CLOCK_DIV14			14
#define RTC_HSE_CLOCK_DIV15			15
#define RTC_HSE_CLOCK_DIV16			16
#define RTC_HSE_CLOCK_DIV17			17
#define RTC_HSE_CLOCK_DIV18			18
#define RTC_HSE_CLOCK_DIV19			19
#define RTC_HSE_CLOCK_DIV20			20
#define RTC_HSE_CLOCK_DIV21			21
#define RTC_HSE_CLOCK_DIV22			22
#define RTC_HSE_CLOCK_DIV23			23
#define RTC_HSE_CLOCK_DIV24			24
#define RTC_HSE_CLOCK_DIV25			25
#define RTC_HSE_CLOCK_DIV26			26
#define RTC_HSE_CLOCK_DIV27			27
#define RTC_HSE_CLOCK_DIV28			28
#define RTC_HSE_CLOCK_DIV29			29
#define RTC_HSE_CLOCK_DIV30			30
#define RTC_HSE_CLOCK_DIV31			31


/*=============================================================================
 * MACROS to enable and disable the various clock oscillators
 * ============================================================================
 */

//for HSE

#define RCC_HSE_ENABLE()						(( RCC ->CR |= (1 << RCC_CR_HSEON) ))

#define RCC_HSE_DISABLE()						(( RCC ->CR &= ~(1 << RCC_CR_HSEON) ))

//for HSI

#define RCC_HSI_ENABLE()						(( RCC ->CR |= (1 << RCC_CR_HSION) ))

#define RCC_HSI_DISABLE()						(( RCC ->CR &= ~(1 << RCC_CR_HSION) ))

//for LSE

#define RCC_LSE_ENABLE()						(( RCC ->BDCR |= (1 << RCC_BDCR_LSEON) ))

#define RCC_LSE_DISABLE()						(( RCC ->BDCR &= ~(1 << RCC_BDCR_LSEON) ))

//for LSI

#define RCC_LSI_ENABLE()						(( RCC ->CSR |= (1 << RCC_CSR_LSION) ))

#define RCC_LSI_DISABLE()						(( RCC ->CSR &= ~(1 << RCC_CSR_LSION) ))

//for RTC

#define RCC_RTC_ENABLE()						(( RCC ->BDCR |= (1 << RCC_BDCR_RTCEN) ))

#define RCC_RTC_DISABLE()						(( RCC ->BDCR &= ~(1 << RCC_BDCR_RTCEN) ))




/*===========================================================================================
 * 						APIs Supported by this driver file
 * 	For details related to API's refer function definition in .c file of the driver
 *==========================================================================================
 */

uint32_t RCC_getPCLK1Value(void);
uint32_t RCC_getPCLK2Value(void);

void RCC_OsclatorConfig(RCC_SYSClkType_Select_t *pOsclator );

void RCC_ClkConfig(RCC_ClkInit_t *pCLKConfig);

void RCC_RTC_Config(RCC_Hanale_t *pRTC);

void RCC_MOC_Config(RCC_MCOx_Config_t *pMCOx );


//LEFT
uint8_t RCC_SystemClkStatus();
void RCC_FlagStatus();

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
