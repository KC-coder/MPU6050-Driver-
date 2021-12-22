
/*
 * stm32F407xx_TIM_driver.h
 *
 *  Created on: 06-Sep-2020
 *      Author: kamal chopra
 */

/*
 * material/text  used to write driver
 * 1.Reference manual
 * 2.AN4013 Application note
 * 3.AN4776 Application note
 * 4.some more application manual
 */

#ifndef INC_STM32F407XX_TIM_DRIVER_H_
#define INC_STM32F407XX_TIM_DRIVER_H_

#include"stm32F407xx.h"

/*
 * configuration structure for the SPIx peripheral
 */

typedef struct
{
  uint32_t IntCountVal;

  uint32_t Prescaler;         /*< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU
                                   as register is of 16 bits*/

  uint32_t CounterMode;       /*!< Specifies the counter mode. NOT FOR BASIC TIMERS
                                    value can be @TIM_Counter_Mode */

  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFF. as
                                   it is of 16bits IF PERIOD VALUE IS 0 TIMER WILL NOT OPERATE */

  uint32_t ClockDivision;     /*!< Specifies the clock division. NOT NEEDED FOR BASIC TIMER
                                   This parameter can be a value of @ref TIM_ClockDivision */

  uint32_t RepetitionCounter;  /*!< NOT NEEDED FOR BASIC TIMERS
  	  	  	  	  	  	  	  	  	 Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.
                                     @note This parameter is valid only for TIM1 and TIM8. */
  uint32_t AutoRelaodBuffer;	/*!< for ARR register as ARR value is in shadow register but we can
  	  	  	  	  	  	  	  	  	  enable a preload(buffer) register if we need so that the value
  	  	  	  	  	  	  	  	  	  we change gets loaded in preload register and gets updated on shadow reigister
  	  	  	  	  	  	  	  	  	   at a update event VALUES for tis : @ref TIM_AutoReoaldBuffer >*/
} TIM_Base_InitTypeDef;



/*
 *  handle structure for SPI Peripheral
 */
typedef struct
{
	TIM_RegDef_t	*pTIMx;			/* This is to hold the base address of SPIx peripherls*/
	TIM_Base_InitTypeDef	TIMConfig; // Common in all timers , it is to configure the base unit of the timers
	uint32_t TIM_ChannelNumber; /*<possible value from @TIM_ChannelNumber>*/


}TIM_Handle_t;

/*
 * structure for general purpose timer
 */
typedef struct
{
	uint32_t IC_filter;		/* !< To store the app. Tx buffer address > */

	uint32_t ICPrescaler;	/* !<possible value from @TIMx_ICPrescaler > */

	uint32_t ICSelection;	/*!<here we do  Capture/Compare input selection,
	 	 	 	 	 	 	 	for possible value ref @TIM_Input_Capture_Selection >*/

	uint32_t ICPolarity;	/*<possible value from @TIMx_ICPolarity>*/

	uint32_t TIM_ChannelNumber; /*<possible value from @TIM_ChannelNumber>*/

}TIM_IC_Init_Handle_t;



typedef struct
{
  uint32_t ClockSource;     /*!< TIM clock sources
                                 This parameter can be a value of @ref TIM_Clock_Source */
  uint32_t ClockPolarity;   /*!< TIM clock polarity
                                 This parameter can be a value of @ref TIM_Clock_Polarity */
  uint32_t ClockPrescaler;  /*!< TIM clock prescaler
                                 This parameter can be a value of ref @TIM_Clock_Prescaler */
  uint32_t ClockFilter;     /*!< TIM clock filter
                                 This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
  uint32_t ICchannelNumber;
} TIM_ClockConfigTypeDef;


typedef struct
{
  uint32_t OC_Mode;        /*!< Specifies the TIM mode.
                               This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

  uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t OC_Polarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref TIM_Output_Compare_Polarity */

  uint32_t OCN_Polarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t OC_FastMode;    /*!< Specifies the Fast mode state.
                               This parameter can be a value of @ref TIM_Output_Fast_State
                               @note This parameter is valid only in PWM1 and PWM2 mode. */


  uint32_t OC_IdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_Output_Compare_Idle_State
                               @note This parameter is valid only for timer instances supporting break feature. */

  uint32_t OCN_IdleState;  /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
                               @note This parameter is valid only for timer instances supporting break feature. */
} TIM_OC_InitTypeDef;



typedef struct
{
  uint32_t  SlaveMode;         /*!< Slave mode selection
                                    This parameter can be a value of @ref TIM_Slave_Mode */
  uint32_t  InputTrigger;      /*!< Input Trigger source
                                    This parameter can be a value of @ref TIM_Trigger_Selection */
  uint32_t  TriggerPolarity;   /*!< Input Trigger polarity
                                    This parameter can be a value of @ref TIM_Trigger_Polarity */
  uint32_t  TriggerPrescaler;  /*!< Input trigger prescaler
                                    This parameter can be a value of @ref TIM_Trigger_Prescaler */
  uint32_t  TriggerFilter;     /*!< Input trigger filter
                                    This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */

} TIM_SlaveConfigTypeDef;

typedef struct
{
	uint8_t InputChannel;

	uint8_t OutputChannel;

	uint32_t ICPolarity;

	uint32_t ICSlection;

	uint32_t ICFilter;

	uint32_t OCPolarity;

	uint32_t OCMode;

	uint32_t Pulse;

	uint32_t OPMode;  //single mode / repetative mode

}TIMx_OnePulse_Init_t;
/*
 * slave controller of the timer has 4 modes of operation
 * this structure is used to configure the selected mode of operation
 */
typedef struct
{
	uint8_t SlaveMode; /*!< to select mode of slave controller
	 	 	 	 	 	 	 values for this ref @@TIM_Slave_Modes >!*/

	uint8_t SlaveInputSource; /*!<to select the input sourcse of the slave controller
								For its parameter ref @TIM_TriggerSelection>!*/

	uint8_t TriggerPolarity; /*!< to select the trigger polarity of the input
	 	 	 	 	 	 	 	 For its parameter ref @TIM_Slave_TriggerPolarity>!*/

	uint8_t TriggerFliter; /*!< To config filter of input source
	 	 	 	 	 	 	  For its parameter ref @TIM_Slave_TriggerPolarity>!*/

	uint8_t TriggerPrescarlar; /*!< to select the trigger polarity of the input
	 	 	 	 	 	 	 	 For its parameter ref @TIM_Slave_TriggerPolarity>!*/
}TIMx_SlaveConfigTypedef;

/*======================================================================================
 *  SOME DRIVER SPECIFIC MACROS
 *======================================================================================
 */


/*For Input capture  prescaler
 *@TIMx_ICPrescaler
 */

#define TIM_IC_PERSC_DIV1			0			 /*!< Capture performed each time an edge is detected on the capture input */
#define TIM_IC_PERSC_DIV2			1			 /*!< Capture performed once every 2 events  */
#define TIM_IC_PERSC_DIV4			2			 /*!< Capture performed once every 2 events */
#define TIM_IC_PERSC_DIV8			3			 /*!< Capture performed once every 2 events  */

/*@TIM_Input_Capture_Selection
 * FOR TIM INPUT CONNECTED TO CH1,CH2,CH3,CH4
 */

#define TIM_ICSELECTION_DIRECTTI 			1	/*!< TIM Input of CHx channel is selected to be connected to ICx  */
#define TIM_ICSELECTION_INDIRECTTI 			2	/*!< TIM Input 1, 2, 3 or 4 is selected to be
                                                     connected to IC2, IC1, IC4 or IC3, respectively */
#define TIM_ICSELECTION_TRC  				3 	/*!< TIM Input selected to be connected to TRC */

/*
 * counter mode selection
 * @TIM_Counter_Mode
 * Note- Up/DOWN mode  configure CMS and DIR bit of TIMx_CR1
 * 	   - for center aligned mode configure CMS bit
 * 	   - BASIC TIMER ALWAYS COUNT UP
 */

#define TIM_COUNTERMODE_UP                 0      /*!< Counter used as up-counter   */
#define TIM_COUNTERMODE_DOWN               1      /*!< Counter used as down-counter */
#define TIM_COUNTERMODE_CENTERALIGNED1     1      /*!< Center-aligned mode 1        */
#define TIM_COUNTERMODE_CENTERALIGNED2     2      /*!< Center-aligned mode 2        */
#define TIM_COUNTERMODE_CENTERALIGNED3     3      /*!< Center-aligned mode 3        */

/*
 * POLARITY, of edge detection
 * @TIMx_ICPolarity
 */
#define TIM_ICPOLARITY_RISING				0	 /*!< Capture triggered by noninverted/rising edge on timer input*/
#define TIM_ICPOLARITY_FALLING				1	 /*!< Capture triggered by inverted/falling edge on timer input*/
#define TIM_ICPOLARITY_BOTHEDGE				3	 /*!< Capture triggered by noninverted/both edges on timer input*/

/*
 * channel number
 *@TIM_ChannelNumber
 */
#define TIM_IC_Channel_1					1
#define TIM_IC_Channel_2					2
#define TIM_IC_Channel_3					3
#define TIM_IC_Channel_4					4


/*
 * interrupt flag name macros used for StatusFlagName
 * @TIM_flag
 */
#define TIM_FLAG_UEV						(1<<TIMx_SR_UIF)
#define TIM_FLAG_IC_OC_CH1					(1<<TIMx_SR_CC1IF)
#define TIM_FLAG_IC_OC_CH2					(1<<TIMx_SR_CC2IF)
#define TIM_FLAG_IC_OC_CH3					(1<<TIMx_SR_CC3IF)
#define TIM_FLAG_IC_OC_CH4					(1<<TIMx_SR_CC4IF)
#define TIM_FLAG_TIG_IT						(1<<TIMx_SR_TIF)
#define TIM_FLAG_OVERCAPTURE_CH1			(1<<TIMx_SR_CC1OF)
#define TIM_FLAG_OVERCAPTURE_CH2			(1<<TIMx_SR_CC2OF)
#define TIM_FLAG_OVERCAPTURE_CH3			(1<<TIMx_SR_CC3OF)
#define TIM_FLAG_OVERCAPTURE_CH4			(1<<TIMx_SR_CC4OF)


/** @defgroup TIM_Clock_Source TIM Clock Source
  * @{
  */

#define TIM_CLOCKSOURCE_EXTMODE1_ITR0	0		/*!< External clock source mode 1 (ITR0)*/
#define TIM_CLOCKSOURCE_EXTMODE1_ITR1	1		 /*!< External clock source mode 1 (ITR1)*/
#define TIM_CLOCKSOURCE_EXTMODE1_ITR2	2		/*!< External clock source mode 1 (ITR2)*/
#define TIM_CLOCKSOURCE_EXTMODE1_ITR3	3		/*!< External clock source mode 1 (ITR3)*/
#define TIM_CLOCKSOURCE_EXTMODE1_TI1ED	4		/*!< External clock source mode 1 (TTI1FP1 + edge detect.) */
#define TIM_CLOCKSOURCE_EXTMODE1_TI1FP1	5		/*!< External clock source mode 1 (TTI1FP1) */
#define TIM_CLOCKSOURCE_EXTMODE1_TI2FP2	6		/*!< External clock source mode 1 (TTI2FP2) */
#define TIM_CLOCKSOURCE_EXTMODE2_ETRF	7		 /*!< External clock source mode 2 (ETRF)  */
#define TIM_CLOCKSOURCE_ETRMODE2		8		/*!< External clock source mode 2  */
#define TIM_CLOCKSOURCE_INTERNAL		9		/*!< Internal clock source  */

/** @defgroup TIM_Clock_Polarity TIM Clock Polarity
  * @{
  */
#define TIM_CLOCKPOLARITY_INVERTED           1      /*!< Polarity for ETRx clock sources */
#define TIM_CLOCKPOLARITY_NONINVERTED        0      /*!< Polarity for ETRx clock sources */
#define TIM_CLOCKPOLARITY_RISING             0   	/*!< Polarity for TIx clock sources */
#define TIM_CLOCKPOLARITY_FALLING            1   	/*!< Polarity for TIx clock sources */
#define TIM_CLOCKPOLARITY_BOTHEDGE           3  	/*!< Polarity for TIx clock sources */


/*
 * timer output compare mode  @TIM_OC_MODE and @TIM_PWM_MODES
 * these mode are set on OCxM bit of TIMx_CCMRx Register
 * this is to define the behavior of the output reference signal
 *for active  OC1REF=1
 *for inactive OC1REF=‘0
 */
#define TIM_OC_MODE_Frozen			0 		/*comparison between the output compare register CCRx and the counter CNT
 	 	 	 	 	 	 	 	 	 	 	  has no effect on the outputs. generate a timing base*/
#define TIM_OC_MODE_Active			1		/*OCxRef signal is forced high, when CNT and CCRx matches*/
#define TIM_OC_MODE_InActive		2		/*OCxRef signal is forced low, when CNT and CCRx matches*/
#define TIM_OC_MODE_Toggle			3		/*OCxRef toggles, when CNT and CCRx matches*/
#define TIM_OC_MODE_ForceInActive	4		/*OC1REF is forced low, when CNT and CCRx matches*/
#define TIM_OC_MODE_ForceActive		5		/*OCREF is forced high, when CNT and CCRx matches*/

#define TIM_OC_MODE_PWM_Mode1		6		/*In upcounting, channel is active as long as TIMx_CNT<TIMx_CCR1 else inactive.
 	 	 	 	 	 	 	 	 	 	 	  In downcounting, channel is inactive as long as TIMx_CNT>TIMx_CCR1 else active*/

#define TIM_OC_MODE_PWM_Mode2		7		/*In upcounting, channel is inactive as long as TIMx_CNT<TIMx_CCR1 else active.
											  In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive*/

/*Capture/Compare  output Polarity
 * configured on bit CCxP of TIMx_CCER register
 * @TIM_Output_Compare_Polarity
 */
#define TIM_OC_Polarity_ActiveHigh			0	/*OCref remains same */
#define TIM_OC_Polarity_ActiveLow			1	/*OCref inverted remains same*/

/*for Output compare 1 preload
 * OC1PE bit of CCMRx
 */
#define TIM_OC_PRELOAD_ENABLE				1
#define TIM_OC_PRELOAD_DISABLED				0

/*
 * output channel number
 *@TIM_ChannelNumber
 */
#define TIM_OC_Channel_1					1
#define TIM_OC_Channel_2					2
#define TIM_OC_Channel_3					3
#define TIM_OC_Channel_4					4



/*
 * @TIM_Slave_Modes
 */

#define TIM_SLAVEMODE_DISABLE 				0
#define TIM_SLAVEMODE_RESET 				4
#define TIM_SLAVEMODE_GATED 				5
#define TIM_SLAVEMODE_TRIGGER 				6
#define TIM_SLAVEMODE_EXTERNAL1 			7

/*
 * @TIM_TriggerSelection
 * trigger inputs  to the slave controller
 */
#define TIM_TRIG_SELECTION_ITR0			0
#define TIM_TRIG_SELECTION_ITR1			1
#define TIM_TRIG_SELECTION_ITR2			2
#define TIM_TRIG_SELECTION_ITR3			3
#define TIM_TRIG_SELECTION_TI1F_ED		4
#define TIM_TRIG_SELECTION_TI1FP1		5
#define TIM_TRIG_SELECTION_TI2FP2		6
#define TIM_TRIG_SELECTION_ETRF			7


/*External trigger prescaler
 * @TIM_Clock_Prescaler
 */

#define TIM_CLOCKPRESCALER_DIV1              0           /*!< No prescaler is used                                                     */
#define TIM_CLOCKPRESCALER_DIV2              1           /*!< Prescaler for External ETR Clock: Capture performed once every 2 events. */
#define TIM_CLOCKPRESCALER_DIV4              2           /*!< Prescaler for External ETR Clock: Capture performed once every 4 events. */
#define TIM_CLOCKPRESCALER_DIV8              3           /*!< Prescaler for External ETR Clock: Capture performed once every 8 events. */

/*
 *  @ref TIM_AutoReoaldBuffer
 *  it is for the ARPE bit of CR1
 *  it is to enable or disable buffer for ARR register
 */
#define TIM_ARR_BUFFER_EN					1
#define TIM_ARR_BUFFER_DI					0

/*
 * some useful macros
 */
#define To_clear_onebit		1
#define To_clear_twobit		3
#define OK					1
#define ERROR				0


/*===========================================================================================
 * 						APIs Supported by this driver file
 * 	For details related to API's refer function definition in .c file of the driver
 *==========================================================================================
 */

/*
 * Peripheral clock setup
 */
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx , uint8_t EnorDi);
void TIM_ClockConfig(TIM_RegDef_t *pTIMx ,TIM_ClockConfigTypeDef *pClockConfig );

/*
 * functions for configuring basic unit of timer
 */
void TIM_BASE_Init(TIM_Handle_t *pTIMHandle);

/*
 * function to enable the timer counter
 * this function has to be called in any mode of the timer
 */
void TIM_COUNTER_EnOrDi(TIM_RegDef_t *pTIMx , uint8_t EnOrDi);

/*
 * input capture  related macros
 *
 */
void TIMx_IC_Config(TIM_RegDef_t*pTIMx , TIM_IC_Init_Handle_t *pTimxHandle );
void TIMx_IC_EnOrDi(TIM_RegDef_t*pTIMx , uint8_t EnOrDi , uint8_t ChannelNumber);
void TIMx_IC_WithIT_EnOrDi(TIM_RegDef_t*pTIMx , uint8_t EnOrDi , uint8_t ChannelNumber);
void TIMx_PWM_IC_Config(TIM_RegDef_t*pTIMx , uint8_t ChannelNumber);
uint32_t TIM_IC_Read(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber);

/*
 * functions for output capture
 */
void TIM_OC_Int(TIM_OC_InitTypeDef *pConfigOC, TIM_RegDef_t *pTIMx , uint8_t ChannelName);
void TIM_OC_EnOrDi(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber ,  uint8_t EnOrDi);
void TIM_OC_EnOrDi_WithIT(TIM_RegDef_t	*pTIMx ,uint8_t ChannelNumber ,  uint8_t EnOrDi);

/*
 * functions for slave controller
 */

void TIM_SlaveController_Int(TIM_RegDef_t	*pTIMx ,TIM_SlaveConfigTypeDef *pSlaveConfig);

/*
 * functions to configure the PWM
 */
void TIM_PWM_Init(TIM_Handle_t *pTimHandle, TIM_OC_InitTypeDef *pConfigOC);

void PWM_Start(TIM_RegDef_t	*pTIMx);

/*
 * functions for one pluse mode
 */
void TIM_OnePulseMode_Init(TIMx_OnePulse_Init_t *pOPMode ,TIM_Handle_t *pTimHandle);
void TIM_OnePulse_Start(TIM_RegDef_t *pTIMx , uint32_t delayValue  );


/*
 * For IRQ configuration and ISR handling
 */

void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void TIM_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void TIM_BASIC_IRQ_HANDLER(TIM_RegDef_t *pTIMx);


/*
 * Other Peripheral control APIs
 */

void TIMx_Error();



#endif /* INC_STM32F407XX_TIM_DRIVER_H_ */
