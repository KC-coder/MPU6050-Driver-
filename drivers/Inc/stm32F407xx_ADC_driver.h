/*
 * stm32F407xx_ADC_driver.h
 *
 *  Created on: 07-Sep-2020
 *      Author: KAMAL CHOPRA
 */

#ifndef INC_STM32F407XX_ADC_DRIVER_H_
#define INC_STM32F407XX_ADC_DRIVER_H_

#include"stm32f407xx.h"

#define ADC1_WATCHDOG_CH4_ENABLE()		( (ADC1->CR1 |= (1<<0) ))
#define ADC1_WATCHDOG_CH4_DISABLE()		( (ADC1->CR1 &=~(1<<0) ))


typedef struct
{
	uint8_t  overrun_IT;				/*<possible value from @overrun_IT >*/
    uint8_t  Resolution;				/*<possible value from @ADC_Resolution >*/
    uint8_t  Analog_watchdog_EnOrDi;	/*<possible value SET OR RESET	 >*/
    uint16_t Analog_watchdog_HT ;		/*<possible value any 12 bit value >*/
    uint16_t Analog_watchdog_LT ;		/*<possible value any 12 bit value >*/
    uint8_t  Discontinuous_Mode_EnOrDi;	/*<possible value SET OR RESET >*/
    uint8_t  Discountionous_Channels;	/*<possible value from @Discountionous_Channels >*/
    uint8_t  Ext_trig_Type;				/*<possible value from @ADC_EXT_TRIG_TYP >*/
    uint16_t Ext_trig_Source;			/*<possible value from @Ext_trig_Source >*/
    uint8_t  Align_Data;				/*<possible value from @ADC_Align_Data >*/
    uint8_t	 Continous_Mode ;
}ADC_Regular_Config_t;



typedef struct
{
	ADC_RegDef_t *ADCx;
	ADC_Regular_Config_t ADC_Config;
	//ADC_Inject_Config_t  ADC_Inject_Config;
	//uint16_t Conversion_mode;
}ADC_Handle_t;

typedef struct
{
	uint8_t wactchdog_IT;
	uint8_t overrun_IT;
	uint8_t EOC_IT;
	uint8_t ECO_Inject_IT;

}ADC_IT_Config;

typedef struct
{
	ADC_RegDef_t *ADCx;
	ADC_IT_Config ADCinterrupt;
}ADC_IT_Handle_t;




/*============================================================================
 * ===========================DRIVER USED MACROS============================*/

/*
 * ADC ON and  OFF macro's
 */
#define ADC1_ON()						((ADC1->CR2 |= (1<<0) ))
#define ADC1_OFF()						((ADC1->CR2 &=~(1<<0) ))

#define ADC2_ON()						((ADC2->CR2 |= (1<<0) ))
#define ADC2_OFF()						((ADC2->CR2 &=~(1<<0) ))

#define ADC3_ON()						((ADC3->CR2 |= (1<<0) ))
#define ADC3_OFF()						((ADC3->CR2 &=~(1<<0) ))




/*@ADC_EXT_TRIG_TYP
 * macros for possible ADCexternal trigger types
 */
#define ADC_EXT_TYP_DIS				0
#define ADC_EXT_TYP_RT				1
#define ADC_EXT_TYP_FT				2
#define ADC_EXT_TYP_RFT				3

/*@Ext_trig_Source
 * macros for possible ADC external trigger sources
 */
#define ADC_EXT_TIM1_CC1			0
#define ADC_EXT_TIM1_CC2			1
#define ADC_EXT_TIM1_CC3			2
#define ADC_EXT_TIM2_CC2			3
#define ADC_EXT_TIM2_CC3			4
#define ADC_EXT_TIM2_CC4			5
#define ADC_EXT_TIM2_TRGO			6
#define ADC_EXT_TIM3_CC1			7
#define ADC_EXT_TIM3_TRGO			8
#define ADC_EXT_TIM4_CC4			9
#define ADC_EXT_TIM5_CC1			10
#define ADC_EXT_TIM5_CC2			11
#define ADC_EXT_TIM5_CC3			12
#define ADC_EXT_TIM8_CC1			13
#define ADC_EXT_TIM8_TRGO			14
#define ADC_EXT_EXTI_11				15

/* @ADC_Resolution
 * Macros for possible resolution in ADC
 */
#define RESOLUTION_12bit		0
#define RESOLUTION_10bit		1
#define RESOLUTION_8bit			2
#define RESOLUTION_6bit			3




/*@Discountionous_Channels
 * ADC_CHANNNELs
 */


#define ADC_CHANNEL_0					0
#define ADC_CHANNEL_1					1
#define ADC_CHANNEL_2					2
#define ADC_CHANNEL_3					3
#define ADC_CHANNEL_4					4
#define ADC_CHANNEL_5					5
#define ADC_CHANNEL_6					6
#define ADC_CHANNEL_7					7
#define ADC_CHANNEL_8					8
#define ADC_CHANNEL_9					9
#define ADC_CHANNEL_10					10
#define ADC_CHANNEL_11					11
#define ADC_CHANNEL_12					12
#define ADC_CHANNEL_13					13
#define ADC_CHANNEL_14					14
#define ADC_CHANNEL_15					15
#define ADC_CHANNEL_16					16
#define ADC_CHANNEL_17					17
#define ADC_CHANNEL_18					18

#define ADC_INJ_CHANNEL_1				1
#define ADC_INJ_CHANNEL_2				2
#define ADC_INJ_CHANNEL_3				3
#define ADC_INJ_CHANNEL_4				4

/*@overrun_IT
 * Over run interrupt macro
 */

#define OVERRUN_IT_EN				1
#define OVERRUN_IT_DI				0

/*@ADC_Align_Data
 *
 */
#define ALIGN_DATA_RIGHT			0
#define ALIGN_DATA_LEFT				1



/*
 * Macros to configure cycles
 */

#define CyclePerSample_3			0
#define CyclePerSample_15			1
#define CyclePerSample_28			2
#define CyclePerSample_56			3
#define CyclePerSample_84			4
#define CyclePerSample_112			5
#define CyclePerSample_114			6
#define CyclePerSample_480			7


// functions to start clock and turn on
void ADC_POWER_EnOrDi(ADC_RegDef_t *pADC , uint8_t EnOrDi);
void ADC_PeriClock(ADC_RegDef_t *pADC , uint8_t EnorDi);


//Function to configure ADC
void ADC_CONFIG_WATCHDOG(ADC_Handle_t *pADC);
void ADC_Channel_Select(ADC_Handle_t *pADCHandle);
void ADC_Int(ADC_Handle_t *pADCHandle);
void ADC_CyclesPerSample(ADC_RegDef_t *pADC , uint16_t channel , uint8_t Cycles);
void ADC_Watchdog_CHSelect(ADC_RegDef_t *ADCx , uint16_t ADC_channnel);
//function to read from ADC

uint16_t ADC_ReadRegularData(ADC_RegDef_t *pADC);
uint16_t ADC_ReadInjectedData(ADC_RegDef_t *pADC , uint8_t Inj_channel);

void ADC_SCAN_MODE(ADC_RegDef_t *ADCx ,uint8_t EnOrDis);

void ADC_Reg_Coversion_EnORDi(ADC_RegDef_t *pADC , uint8_t EnOrDi);


void ADC_RegularChannelConfig(ADC_RegDef_t *pADC, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);

/*
 * conversion mode
 */

#define Independent_Mode
#define Dual_Mode
#define Combined_RegSimun_InjSimun
#define Combined_RegSimun_altTrig
#define InjSimun
#define RegularSimun

#define Continous_Conv_Mode
#define Scan_Mode


#define Triggered_injection
#define Auto_injection





#endif /* INC_STM32F407XX_ADC_DRIVER_H_ */
