/*
 * stm32F407xx_SPI_driver.h
 *
 *  Created on: 01-Sep-2020
 *      Author: KAMAL
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include"stm32F407xx.h"

/*
 * configuration structure for the SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;				/*<possible value from @SPI_Device_Modes >*/
	uint8_t SPI_BusConfig;				/*<possible value from @SPI_BusConfig >*/
	uint8_t SPI_SclkSpeed;				/*<possible value from @SPI_SclkSpeed >*/
	uint8_t SPI_DFF;					/*<possible value from @SPI_DFF >*/
	uint8_t SPI_CPOL;					/*<possible value from @SPI_CPOL >*/
	uint8_t SPI_CPAH;					/*<possible value from @SPI_SPI_CPAH >*/
	uint8_t SPI_SSM;					/*<possible value from @SPI_SSM >*/

}SPI_Config_t;

/*
 *  handle structure for SPI Peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;			/* This is to hold the base address of SPIx peripherls*/
	SPI_Config_t	SPIConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxState;	/* !< To store Tx state > */
	uint8_t 		RxState;	/* !< To store Rx state > */

}SPI_Handle_t;

/*======================================================================================
 *  SOME DRIVER SPECIFIC MACROS
 *======================================================================================
 */

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4


/*
 * @SPI_Device_Modes
 * To configure MSTR bit of SPI_CR1 register
 */

#define SPI_DEVICE_MODE_MASTER					1
#define SPI_DEVICE_MODE_SLAVE					0

/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD						1	//Full duplex
#define SPI_BUS_CONFIG_HD						2	//Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3

/*
 * @SPI_SclkSpeed
 * Used to configure BR (baud rate) bit of CR1 , these are to divides the
 * peripheral clock
 */

#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1

/*
 * @SPI_SPI_CPAH
 */

#define SPI_CPAH_LOW							0
#define SPI_CPAH_HIGH							1

/*
 * @SPI_SSM
 */

#define SPI_SSM_DI								0
#define SPI_SSM_EN								1

/*
 * FLAG name macros
 */

#define SPI_TXE_FLAG							(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG							(1 << SPI_SR_RXNE)
#define SPI_CHSIDE_BSY							(1 << SPI_SR_CHSIDE)
#define SPI_UDR_BSY								(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG							(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG							(1 << SPI_SR_MODF)
#define SPI_OVR_BSY								(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG							(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG							(1 << SPI_SR_FRE)




/*===========================================================================================
 * 						APIs Supported by this driver file
 * 	For details related to API's refer function definition in .c file of the driver
 *==========================================================================================
 */


/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);

/*
 * For initialization and deinitialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInt(SPI_RegDef_t *pSPIx ); // for this we have a reset register in RCC section to reset a GPIO


/*
 * Data send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer, uint32_t Len);//pTxBuffer is the pointer to the data
void SPI_ReceiveData(SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * For IRQ configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

void SPI_IRQHandling(SPI_Handle_t *PHandle);

void SPI_IRQPriorityConfig (uint8_t IRQNumber , uint32_t IRQPriority);


/*
 * Other Peripheral control APIs
 */
 void SPI_PeripheralControl(SPI_RegDef_t *pSPIx ,uint8_t EnorDi);

 void SPI_SSIConfig(SPI_RegDef_t *pSPIx ,uint8_t EnorDi);

 void SPI_SSOEConfig(SPI_RegDef_t *pSPIx ,uint8_t EnorDi);

 uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

 void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

 void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);

 void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


 /*
  * appliction call back
  *
  */
 void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
