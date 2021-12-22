/*
 * MPU6050_Driver.h
 *
 *  Created on: Jul 25, 2021
 *  Last revision : 06/08/21
 *      Author: kamal chopra
 */
// RADDR		: register address

#ifndef MPU6050_DRIVER_H_
#define MPU6050_DRIVER_H_

#include <stdio.h>
#include <stdint.h>


#include "stm32F407xx_I2C_driver.h"

I2C_Handle_t *System_I2C_ForMPU;

/*Register address */

//register for self test
#define MPU6050_SELF_TEST_X_REG_ADDR			0x0D
#define MPU6050_SELF_TEST_Y_REG_ADDR			0x0E
#define MPU6050_SELF_TEST_Z_REG_ADDR			0x0F
#define MPU6050_SELF_TEST_A_REG_ADDR			0x10


#define MPU6050_SMPLRT_DIV_REG_ADDR				0x19
#define MPU6050_CONFIG_REG_ADDR					0x1A
#define MPU6050_GYRO_CONFIG_REG_ADDR			0x1B
#define MPU6050_ACCEL_CONFIG_REG_ADDR			0x1C
#define MPU6050_MOT_THR_REG_ADDR				0x1F
#define MPU6050_FIFO_EN_REG_ADDR				0x23
#define MPU6050_I2C_MST_CTRL_REG_ADDR			0x24

/* Registers for slave0 */
#define MPU6050_I2C_SLV0ADDR_REG_ADDR			0x25
#define MPU6050_I2C_SLV0REG_REG_ADDR			0x26
#define MPU6050_I2C_SLV0CTRL_REG_ADDR			0x27

/* Registers for slave1 */
#define MPU6050_I2C_SLV1ADDR_REG_ADDR			0x28
#define MPU6050_I2C_SLV1REG_REG_ADDR			0x29
#define MPU6050_I2C_SLV1CTRL_REG_ADDR			0x2A

/* Registers for slave2 */
#define MPU6050_I2C_SLV2ADDR_REG_ADDR			0x2B
#define MPU6050_I2C_SLV2REG_REG_ADDR			0x2C
#define MPU6050_I2C_SLV2CTRL_REG_ADDR			0x2D

/* Registers for slave3 */
#define MPU6050_I2C_SLV3ADDR_REG_ADDR			0x2E
#define MPU6050_I2C_SLV3REG_REG_ADDR			0x2F
#define MPU6050_I2C_SLV3CTRL_REG_ADDR			0x30

/* Register for slave4 */
#define MPU6050_I2C_SLV4ADDR_REG_ADDR			0x31
#define MPU6050_I2C_SLV4REG_REG_ADDR			0x32
#define MPU6050_I2C_SLV4DO_REG_ADDR				0x33
#define MPU6050_I2C_SLV4CTRL_REG_ADDR			0x34
#define MPU6050_I2C_SLV4DI_REG_ADDR				0x35

#define MPU6050_I2C_MST_STATUS_REG_ADDR			0x36
#define MPU6050_INT_PIN_CFG_REG_ADDR			0x37
#define MPU6050_INT_ENABLE_REG_ADDR				0x38
#define MPU6050_INT_STATUS_REG_ADDR				0x3A

/* Registers for Accelerometer data*/
#define MPU6050_ACCEL_XOUT_H_REG_ADDR			0x3B
#define MPU6050_ACCEL_XOUT_L_REG_ADDR			0x3C
#define MPU6050_ACCEL_YOUT_H_REG_ADDR			0x3D
#define MPU6050_ACCEL_YOUT_L_REG_ADDR			0x3E
#define MPU6050_ACCEL_ZOUT_H_REG_ADDR			0x3F
#define MPU6050_ACCEL_ZOUT_L_REG_ADDR			0x40

/* Registers for temp data*/
#define MPU6050_TEMP_OUT_H_REG_ADDR				0x41
#define MPU6050_TEMP_OUT_L_REG_ADDR				0x42

/* Registers for Gyroscope data*/
#define MPU6050_GYRO_XOUT_H_REG_ADDR			0x43
#define MPU6050_GYRO_XOUT_L_REG_ADDR			0x44
#define MPU6050_GYRO_YOUT_H_REG_ADDR			0x45
#define MPU6050_GYRO_YOUT_L_REG_ADDR			0x46
#define MPU6050_GYRO_ZOUT_H_REG_ADDR			0x47
#define MPU6050_GYRO_ZOUT_L_REG_ADDR			0x48

/* Registers to store  slave data ( i.e data from external sensor ) */
#define MPU6050_EXT_SENS_DATA_00_REG_ADDR		0x49
#define MPU6050_EXT_SENS_DATA_01_REG_ADDR		0x4A
#define MPU6050_EXT_SENS_DATA_02_REG_ADDR		0x4B
#define MPU6050_EXT_SENS_DATA_03_REG_ADDR		0x4C
#define MPU6050_EXT_SENS_DATA_04_REG_ADDR		0x4D
#define MPU6050_EXT_SENS_DATA_05_REG_ADDR		0x4E
#define MPU6050_EXT_SENS_DATA_06_REG_ADDR		0x4F
#define MPU6050_EXT_SENS_DATA_07_REG_ADDR		0x50
#define MPU6050_EXT_SENS_DATA_08_REG_ADDR		0x51
#define MPU6050_EXT_SENS_DATA_09_REG_ADDR		0x52
#define MPU6050_EXT_SENS_DATA_10_REG_ADDR		0x53
#define MPU6050_EXT_SENS_DATA_11_REG_ADDR		0x54
#define MPU6050_EXT_SENS_DATA_12_REG_ADDR		0x55
#define MPU6050_EXT_SENS_DATA_13_REG_ADDR		0x56
#define MPU6050_EXT_SENS_DATA_14_REG_ADDR		0x57
#define MPU6050_EXT_SENS_DATA_15_REG_ADDR		0x58
#define MPU6050_EXT_SENS_DATA_16_REG_ADDR		0x59
#define MPU6050_EXT_SENS_DATA_17_REG_ADDR		0x5A
#define MPU6050_EXT_SENS_DATA_18_REG_ADDR		0x5B
#define MPU6050_EXT_SENS_DATA_19_REG_ADDR		0x5C
#define MPU6050_EXT_SENS_DATA_20_REG_ADDR		0x5D
#define MPU6050_EXT_SENS_DATA_21_REG_ADDR		0x5E
#define MPU6050_EXT_SENS_DATA_22_REG_ADDR		0x5F
#define MPU6050_EXT_SENS_DATA_23_REG_ADDR		0x60

/* Register to put address of the slaves */
#define MPU6050_I2C_SLV0_DO_REG_ADDR			0x63
#define MPU6050_I2C_SLV1_DO_REG_ADDR			0x64
#define MPU6050_I2C_SLV2_DO_REG_ADDR			0x65
#define MPU6050_I2C_SLV3_DO_REG_ADDR			0x66

#define MPU6050_I2C_MSTDELAY_CTRL_REG_ADDR		0x67
#define MPU6050_SIGNAL_RESET_REG_ADDR			0x68
#define MPU6050_MOTION_CTRL_REG_ADDR			0x69
#define MPU6050_USER_CTRL_REG_ADDR				0x6A
#define MPU6050_PWR_MGMT_1_REG_ADDR				0x6B
#define MPU6050_PWR_MGMT_2_REG_ADDR				0x6C
#define MPU6050_FIFO_COUNTH_REG_ADDR			0x72
#define MPU6050_FIFO_COUNTL_REG_ADDR			0x73
#define MPU6050_FIFO_R_W_REG_ADDR				0x74
#define MPU6050_WHO_AM_I_REG_ADDR				0x75



/*=========================================================================================
 *  					MPU6050 register's bit definition
 * =========================================================================================
 */

/*
 * Of register 26 : configuration register
 */
#define MPU6050_CFG_EXT_SYNC_SET_BIT			3
#define MPU6050_CFG_DLPF_COFIG_BIT				0

/*
 * For register 27 - Guroscope configuration register
 * @GYRO_CONFIG
 */
#define MPU6050_GYRO_FS_SEL_BIT					3
#define MPU6050_GYRO_XG_ST_BIT					5
#define MPU6050_GYRO_YG_ST_BIT					6
#define MPU6050_GYRO_ZG_ST_BIT					7


/*
 * For register 28 - accelerometer configuration
 * @ACCEL_CONFIG
 */
#define	MPU6050_ACCEL_AFS_SEL_BIT				3
#define	MPU6050_ACCEL_XA_ST_BIT					5
#define	MPU6050_ACCEL_YA_ST_BIT					6
#define	MPU6050_ACCEL_ZA_ST_BIT					7



/*
 * For Register 35 - FIFO enable
 * @FIFO_EN
 */
#define MPU6050_SLV0_FIFO_EN_BIT				0
#define MPU6050_SLV1_FIFO_EN_BIT				1
#define MPU6050_SLV2_FIFO_EN_BIT				2
#define MPU6050_ACCEL_FIFO_EN_BIT				3
#define MPU6050_ZG_FIFO_EN_BIT					4
#define MPU6050_YG_FIFO_EN_BIT					5
#define MPU6050_XG_FIFO_EN_BIT					6
#define MPU6050_TEMP_FIFO_EN_BIT				7


/*
 * For Register 36 - I2C Master Control
 * @I2C_MST_CTRL
 */
#define MPU6050_I2C_MST_CLK_BIT					0
#define MPU6050_I2C_MST_PNSR_BIT				4
#define MPU6050_SLV3_FIFO_EN_BIT				5
#define MPU6050_I2C_WAIT_FIFO__EN_BIT			6
#define MPU6050_I2C_MULT_MAST_EN_BIT			7


/*
 * For register 36 to 39 - I2C Slave 0 Control
 */
#define MPU6050_I2C_SLV0_RW_BIT					7
#define MPU6050_I2C_SLV0_ADDR_BIT				0
#define MPU6050_I2C_SLV0_DATALEN_BIT			0
#define MPU6050_I2C_SLV0_GRP_BIT				4
#define MPU6050_I2C_SLV0_REG_DIS_BIT			5
#define MPU6050_I2C_SLV0_BYTE_SW_BIT			6
#define MPU6050_I2C_SLV0_EN_BIT					7


/*
 * For register 40 to 42 - I2C Slave 1 Control
 */
#define MPU6050_I2C_SLV1_RW_BIT					7
#define MPU6050_I2C_SLV1_ADDR_BIT				0
#define MPU6050_I2C_SLV1_DATALEN_BIT			0
#define MPU6050_I2C_SLV1_GRP_BIT				4
#define MPU6050_I2C_SLV1_REG_DIS_BIT			5
#define MPU6050_I2C_SLV1_BYTE_SW_BIT			6
#define MPU6050_I2C_SLV1_EN_BIT					7


/*
 * For register 43 to 45 - I2C Slave 2 Control
 */
#define MPU6050_I2C_SLV2_RW_BIT					7
#define MPU6050_I2C_SLV2_ADDR_BIT				0
#define MPU6050_I2C_SLV2_DATALEN_BIT			0
#define MPU6050_I2C_SLV2_GRP_BIT				4
#define MPU6050_I2C_SLV2_REG_DIS_BIT			5
#define MPU6050_I2C_SLV2_BYTE_SW_BIT			6
#define MPU6050_I2C_SLV2_EN_BIT					7


/*
 * For register 46 to 48 - I2C Slave 3 Control
 */
#define MPU6050_I2C_SLV3_RW_BIT					7
#define MPU6050_I2C_SLV3_ADDR_BIT				0
#define MPU6050_I2C_SLV3_DATALEN_BIT			0
#define MPU6050_I2C_SLV3_GRP_BIT				4
#define MPU6050_I2C_SLV3_REG_DIS_BIT			5
#define MPU6050_I2C_SLV3_BYTE_SW_BIT			6
#define MPU6050_I2C_SLV3_EN_BIT					7

/*
 * For register 49 to 53 - I2C Slave 4 Control
 */
#define MPU6050_I2C_SLV4_RW_BIT					7
#define MPU6050_I2C_SLV4_ADDR_BIT				0
#define MPU6050_I2C_SLV4_REG_DIS_BIT			5
#define MPU6050_I2C_SLV4_INT_EN_BIT				6
#define MPU6050_I2C_SLV4_EN_BIT					7

/*
 * For  Register 55 - INT pin / Bypass Enable Configuration
 * @INT_PIN_CFG
 */
#define MPU6050_I2C_BYPASS_EN_BIT				1
#define MPU6050_FSYNC_IT_EN_BIT					2
#define MPU6050_FSYNC_INT_LEVEL_BIT				3
#define MPU6050_INT_PIN_RD_CLEAR_BIT			4
#define MPU6050_INT_PIN_LATCH_EN_BIT			5
#define MPU6050_INT_PIN_TYPE_BIT				6
#define MPU6050_INT_PIN_LEVEL_BIT				7

/*
 * For Register 56 - Interrupt Enable
 * @INT_ENABLE
 */
#define MPU6050_DATA_RDY_IT_EN_BIT				0
#define MPU6050_I2C_MST_IT_EN_BIT				3
#define MPU6050_FIFO_OFLOW_IT_EN_BIT			4
#define MPU6050_MOTION_DET_IT_EN_BIT			6

/*
 * For Register 58 - Interrupt status
 * @INT_STATUS
 */
#define MPU6050_DATA_RDY_IT_FLAG_BIT			0
#define MPU6050_I2C_MST_IT_FLAG_BIT				3
#define MPU6050_FIFO_OFLOW_IT_FLAG_BIT			4
#define MPU6050_MOTION_DET_IT_FLAG_BIT			6

/* For Register 103 - I2C master delay Control
 *  I2C_MST_DELAY_CTRL
 */
#define MPU6050_I2C_SLV0_DLY_EN_BIT				0
#define MPU6050_I2C_SLV1_DLY_EN_BIT				1
#define MPU6050_I2C_SLV2_DLY_EN_BIT				2
#define MPU6050_I2C_SLV3_DLY_EN_BIT				3
#define MPU6050_I2C_SLV4_DLY_EN_BIT				4
#define MPU6050_DLY_EXTSENS_SHADOW_EN_BIT		0

/*
 * For Register 104 - Signal path Reset
 * @SIGNAL_PATH_RESET
 */
#define MPU6050_TEMP_SIGNAL_RESET_BIT			0
#define MPU6050_ACCEL_SIGNAL_RESET_BIT			1
#define MPU6050_GYRO_SIGNAL_RESET_BIT			2



/*
 * For Register 106 - User Control
 * @USER_CTRL
 */
#define MPU6050_SIG_COND_RESET_BIT				0 /*<>*/
#define MPU6050_I2C_MST_RESET_BIT				1 /*<>*/
#define MPU6050_FIFO_RESET_BIT					2 /*<>*/
#define MPU6050_I2C_MSTER_EN_BIT				5 /*<To Enable or disable the master mode of I2C>*/
#define MPU6050_FIFO_EN_BIT						6/*<To Enable/Disable FIFO buffer>*/


/*
 * For Register 107 - Power Management1
 * @PWR_MGMT_1
 */
#define MPU6050_CKL_SELECT_BIT					0
#define MPU6050_TEMP_SENSOR_DI_BIT				3
#define MPU6050_CYCLE_EN_BIT					5
#define MPU6050_SLEEP_EN_BIT					6
#define MPU6050_DEVICE_RESET_BIT				7

/*
 * For Register 107 - Power Management2
 * @PWR_MGMT_12
 *In this register we can set the required axis of gyro/accelerometer on standby
 *In this register we could config the wake up frequency of the accelerometer low power mode
 */
#define MPU6050_STBY_ZG_EN_BIT					0
#define MPU6050_STBY_YG_EN_BIT					1
#define MPU6050_STBY_XG_EN_BIT					2
#define MPU6050_STBY_ZA_EN_BIT					3
#define MPU6050_STBY_YA_EN_BIT					4
#define MPU6050_STBY_XA_EN_BIT					5
#define MPU6050_LOWPOW_WAKE_CTRL_FRQ_BIT		6











/* Macros for configuring the bits */


/*
 * for Configuring the external
 */
#define MPU6050_EXT_SYNC_DISABLE				0
#define	MPU6050_EXT_SYNC_TEMP_OUT_L				1
#define	MPU6050_EXT_SYNC_GYRO_XOUT_L			2
#define	MPU6050_EXT_SYNC_GYRO_YOUT_L			3
#define	MPU6050_EXT_SYNC_GYRO_ZOUT_L			4
#define	MPU6050_EXT_SYNC_ACCEL_XOUT_L			5
#define	MPU6050_EXT_SYNC_ACCEL_YOUT_L			6
#define	MPU6050_EXT_SYNC_ACCEL_ZOUT_L			7

/*
 * For configuring digital low pass filter DLPF
 * DLPFvalue is same for both accelometer and the gyroscope
 */

#define MPU6050_DLPF_BW_5						6
#define MPU6050_DLPF_BW_10						5
#define MPU6050_DLPF_BW_21						4
#define MPU6050_DLPF_BW_44						3
#define MPU6050_DLPF_BW_94						2
#define MPU6050_DLPF_BW_184						1
#define MPU6050_DLPF_BW_260						0

/*
 * For configuring the full scale range of GYRO
 */

#define MPU6050_GYRO_FS_250						0
#define MPU6050_GYRO_FS_500						1
#define MPU6050_GYRO_FS_1000					2
#define MPU6050_GYRO_FS_2000					3

/*
 * For configuring the full scale range of ACCEL
 */

#define	MPU6050_ACCEL_FS_2						0
#define	MPU6050_ACCEL_FS_4						1
#define	MPU6050_ACCEL_FS_8						2
#define	MPU6050_ACCEL_FS_16						3

/* @MasterClkSp
 * For  configuring the I2C_MST_CLK bits of I2C_MST_CTRL register
 * It configures the divider on the MPU_60X0 internal 8Mhz clock
 * By configuring the divider the clock of the MPU is set to this value
 */

#define MPU6050_I2C_MST_CLK_348					0
#define MPU6050_I2C_MST_CLK_333					1
#define MPU6050_I2C_MST_CLK_320					2
#define MPU6050_I2C_MST_CLK_308					3
#define MPU6050_I2C_MST_CLK_296					4
#define MPU6050_I2C_MST_CLK_286					5
#define MPU6050_I2C_MST_CLK_276					6
#define MPU6050_I2C_MST_CLK_267					7
#define MPU6050_I2C_MST_CLK_258					8
#define MPU6050_I2C_MST_CLK_500					9
#define MPU6050_I2C_MST_CLK_471					10
#define MPU6050_I2C_MST_CLK_444					11
#define MPU6050_I2C_MST_CLK_421					12
#define MPU6050_I2C_MST_CLK_400					13
#define MPU6050_I2C_MST_CLK_381					14
#define MPU6050_I2C_MST_CLK_364					15



/*
 * For selecting the clock source for the MPU
 */

#define MPU6050_CLK_INTERNAL					0
#define	MPU6050_CLK_PLL_WITH_XGYROREF			1
#define	MPU6050_CLK_PLL_WITH_YGYROREF			2
#define	MPU6050_CLK_PLL_WITH_ZGYROREF			3
#define	MPU6050_CLK_PLL_WITH_32768HzREF			4
#define	MPU6050_CLK_PLL_WITH_19200KHzREF		5
#define	MPU6050_CLK_STOPED						7

/*
 *  as we can configure the frequency of wake ups in Acceleromter
 * only in low power mode
 * For configuring the LP_WAKE_CTRL	 	bit to set the
 * wake up frequency
 */

#define MPU6050_LP_WAKE_UP_FEQ_1HZ				0 /*< here frequency is  1.25hZ not 1Hz>*/
#define MPU6050_LP_WAKE_UP_FEQ_5HZ				1
#define MPU6050_LP_WAKE_UP_FEQ_20HZ				2
#define MPU6050_LP_WAKE_UP_FEQ_40HZ				3


/*other macros */

#define DISABLE									0
#define LOW										DISABLE
#define ENABLE									1
#define HIGH									ENABLE

// @SlaveName
#define MPU6050_GYRO_X_AXIS						0
#define MPU6050_GYRO_Y_AXIS						1
#define MPU6050_GYRO_Z_AXIS						2
#define MPU6050_ACCEL_X_AXIS					3
#define MPU6050_ACCEL_Y_AXIS					4
#define MPU6050_ACCEL_Z_AXIS					5
#define MPU6050_SLAVE_0							6
#define MPU6050_SLAVE_1							7
#define MPU6050_SLAVE_2							8
#define MPU6050_SLAVE_3							9
#define MPU6050_SLAVE_4							10


//for INT pin (pg27)

//@ToConfig_INT_PIN_Level
#define MPU6050_INT_PIN_ACTIVE_HIGH				0 //the logic level for the INT pin is active high
#define MPU6050_INT_PIN_ACTIVE_LOW				1 //the logic level for the INT pin is active low

//@ToConfig_INT_PIN_Type
#define MPU6050_INT_PIN_MODE_PP					0 //the INT pin is configured as push-pull
#define MPU6050_INT_PIN_MODE_OD					1 //the INT pin is configured as open drain

//@ToConfig_INT_PIN_StatewhenInterrupt ouccrs
#define MPU6050_INT_PIN_LATCHED					1 //the INT pin is held high until the interrupt is cleared
#define MPU6050_INT_PIN_NOT_LATCHED				0 //the INT pin emits a 50us long pulse WHEN INTERRUPT OUCCERS

//@methodToClear_INT_PIN
#define MPU6050_INT_PIN_ANY_READ				0 //interrupt status bits are cleared only by reading INT_STATUS
#define MPU6050_INT_PIN_STATUS_READ				1 //interrupt status bits are cleared on any read operation

//interrupt names
#define MPU6050_DATA_READY_IT					0
#define MPU6050_I2C_MASTER_IT					1
#define MPU6050_FIFO_OFLOW_IT					2
#define MPU6050_MOTION_DETECTION_IT				3

//address of MPU60x0 REFER TO @address of I2C
#define MPU6050_I2C_ADDRESS_A0					0x68
#define MPU6050_I2C_ADDRESS_A1					0x69

/*
 * #define MPU6050_SLAVE0_FIFO_EN					(MPU6050_FIFO_EN_REG_ADDR |= (ENABLE << ))
 * THIS can't be used as this is a sensor we config it by sending data through the I2C to the sensor register
 * it would have been valid if MPU6050_FIFO_EN_REG_ADDR is present in the MCU
 *
 */
/*=======================================================================================================================================
 * 		MPU6050 : DIVER API's
 * ===================================================================================================================================
 */


typedef struct
{
	uint8_t SlaveName ; /*< Here we have to write which slave we are working on i.e Slave 1 ,2 ,3 or 4 >*/

	uint8_t	SlvAddr;	/*< Here, we write the address of the slave we are working on i.e the I2C address
							of the Sensor which act as a slave  >*/

	uint8_t SlvBytSwap; /*<  When set to 1, this bit enables byte swapping
							When byte swapping is enabled, the high and low bytes of a word pair are swapped
							When cleared to 0, bytes transferred to and from Slave 0 will be written to
							EXT_SENS_DATA registers in the order they were transferred.>*/

	uint8_t SlaveFIFO_EnorDi;/*< To Enable/ Disable FIFO for that salveX>*/

	uint8_t SlvRegAddr;		/*<8-bit address of the SlaveX internal register to/from which data
							 transfer starts>*/

	uint8_t GroupOder;		/*<GroupOder specifying the grouping order of word pairs received from registers
							   When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc
							   (even, then odd register addresses) are paired to form a word.
							   When set to 1, bytes from register addresses are paired 1 and 2, 3 and 4, etc.
							   (odd, then even register addresses) are paired to form a word.>*/

	uint8_t SlvOut_dataLen;/*< 4-bit unsigned value. Specifies the number of bytes transferred to and from Slave
							 Clearing this bit to 0 is equivalent to disabling the register
							  by writing 0 to I2C_SLV0_EN.>*/

	uint8_t Slv_REG_DIS; /*< When set to 1, the transaction will read or write data only
							 When cleared to 0, the transaction will write a register
							 address prior to reading or writing data
							 has to be configured based on the salve connected >*/
}MPU6050_SlaveHandle_struct_t;

typedef struct
{
	uint8_t PinLevel;		 /*<@ToConfig_INT_PIN_Level>*/
	uint8_t PinType; 		/*<@ToConfig_INT_PIN_Type >*/
	uint8_t OutState;			/*<//@ToConfig_INT_PIN_StatewhenInterrupt ouccrs>*/
	uint8_t PinClearType; 	/*<//@methodToClear_INT_PIN>*/

}MPU6050_IntPinHandle_t;
typedef struct
{
	uint8_t DLF_Val;
	uint8_t FS_Val;
	uint8_t Gyro_FIFO_EnOrDi;

}MPU6050_GyroHandle_Struct_t;

typedef struct
{
	uint8_t DLF_Val;
	uint8_t FS_Val;
	uint8_t FIFO_EnOrDi;
	uint8_t Motion_thershold;		//needed if we use motion detection interrupt



}MPU6050_AccelHandle_Struct_t;

typedef struct
{
	uint8_t MPU_ADDR ;
	uint8_t DLPF_Val;
	uint8_t Accel_FS_Val;
	uint8_t Gyro_FS_Val;
	uint8_t Accel_FIFO_EnOrDi;
	uint8_t Gyro_FIFO_EnOrDi;
	uint8_t Temp_EnOrDi;
	uint8_t Temp_FIFO_EnOrDi;
	uint8_t SampleRateDiv ;

}MPU6050_Handle_struct_t;

//================================================================================

/*
 * MPU6050 : Device Related functions
 */


void MPU6050_Set_ADDR(uint8_t address);
void MPU6050_Sleep_disable();
void MPU6050_Reset();
void MPU6050_DLPF_Config(uint8_t DLPFVal);
void MPU6050_FSYNC_Config(uint8_t FSYNCVal);

uint8_t MPU6050_DLPF_GetVal();
uint8_t MPU6050_FSYNC_GetVal();
uint8_t MPU6050_GetMPU_ADDR();


/*
 *  functions for self testing  for Self testing
 */

uint8_t MPU6050_SelfTest_Config();

//function to self test individual GYRO axis
uint8_t MPU6050_XGyro_SelfTest();
uint8_t MPU6050_YGyro_SelfTest();
uint8_t MPU6050_ZGyro_SelfTest();

//function to self test individual ACCEL axis
uint8_t MPU6050_XAccel_SelfTest();
uint8_t MPU6050_YAccel_SelfTest();
uint8_t MPU6050_ZAccel_SelfTest();

uint8_t MPU6050_XAccel_SelfTest_data();
uint8_t MPU6050_YAccel_SelfTest_data();
uint8_t MPU6050_ZAccel_SelfTest_data();

uint8_t MPU6050_XGyro_SelfTest_data();
uint8_t MPU6050_YGyro_SelfTest_data();
uint8_t MPU6050_ZGyro_SelfTest_data();

/*
 * functions for Sample rate of MPU
 */
void MPU6050_SampleRate_DivConfig(uint8_t dividerVal);
uint8_t MPU6050_GetSampleRateVal();

void MPU6050_AccelGyroTemp_Config(MPU6050_Handle_struct_t *pConfig);
void MPU6050_Accel_Config(MPU6050_AccelHandle_Struct_t *Accel_config);
void MPU6050_Gyro_Config();



/*
 * For configuring Slave control features of the MPU
 */
void MPU6050_SlaveX_Config();
void MPU6050_Set_SlaveX_DataLn(uint8_t SlaveName ,uint8_t Len);//TODO ADD Comments
void MPU6050_Get_SlaveX_DataLn(uint8_t SlaveName ,uint8_t Len);//TODO + Comments
/*
 * for configuring the ACCelerometer/Gyro/temp sensor of the MPU-6050
 */
void MPU6050_GyroFS_Config(uint8_t FS_val);
void MPU6050_AccelFS_Config(uint8_t FS_val);
//amke function to get FS range value

/*
 * for reading  Accelerometer  data
 */
void MPU6050_Get_AccelData(uint16_t *Xaccel ,uint16_t *Yaccel ,uint16_t *Zaccel);
void MPU6050_Get_X_AccelData(uint16_t *Xaccel );
void MPU6050_Get_Y_AccelData(uint16_t *Yaccel );
void MPU6050_Get_Z_AccelData(uint16_t *Zaccel);

/*
 * For Reading Gyroscope data
 *
 */
void MPU6050_Get_GyroData(uint16_t *Xgyro ,uint16_t *Ygyro ,uint16_t *Zgyro);
void MPU6050_Get_X_GyroData(uint16_t *Xgyro );
void MPU6050_Get_Y_GyroData(uint16_t *Ygyro );
void MPU6050_Get_Z_GyroData(uint16_t *Zgyro);

/*
 * for salve data
 */
void MPU6050_Get_TempData(uint16_t *TempVal);
void MPU6050_Write_SlavexData(uint8_t data ,uint8_t SlaveName );//todo change R/W bit
void MPU6050_Get_SlavexData(uint16_t SlaveName );//TODO
/*
 * other helper functions
 */
void MPU6050_ClkSource_Select(uint8_t ClkName);
/*
 * Functions related to the FIFO
 */
void MPU6050_Slave_EnOrDi(uint8_t SlaveName , uint8_t EnOrDi );
void MPU6050_FIFO_GyroEnOrDi(uint8_t GyroAxis , uint8_t EnOrDi );
void MPU6050_FIFO_SlaveEnOrDi(uint8_t SlaveName , uint8_t EnOrDi );
void MPU6050_FIFO_AccelEnOrDi( uint8_t EnOrDi ); // add f=FIFO ENABLE BIT
// make FIFO for temp sensor
//make function if fifo bit is enable or not


void MPU6050_MotDet_Config(uint8_t thresVal);

/*
 * communication I2C
 */
void MPU6050_I2C_BYPASS(uint8_t EnOrDi);
void MPU6050_I2C_MasterCtrl_config();//TODO i.e to config the master I2C of the sensor
void MPU6050_I2C_MasterClk_Val(uint8_t MasterClkSp); // TODO to make read master clk val
void MPU6050_I2C_Multi_EnOrDi(uint8_t EnOrDi);
// make function
//Multi-master capability allows multiple I2C masters to operate on the same
//bus. In circuits where multi-master capability is required, set M

/*
 * for configuring the interrupts
 */

void MPU6050_IntPin_Config(MPU6050_IntPinHandle_t *pHandle );//TODO comments
void MPU6050_IntPin_LevelConfig(uint8_t highOrLow);
void MPU6050_IntPin_ModeConfig(uint8_t Mode);
void MPU6050_IntPin_LatchEnOrDi(uint8_t EnOrDi);
void MPU6050_FSYNC_INTLevel(uint8_t highOrLow);
uint8_t MPU6050_GetIT_Status(uint8_t InteruptName);
void MPU6050_ITx_EnOrDi(uint8_t InteruptName , uint8_t EnOrDi);


/*
 * other reset related functions
 */

void MPU6050_AccelSenor_Reset();
void MPU6050_GyroSenor_Reset();
void MPU6050_TempSenor_Reset();
void MPU6050_FIFO_Reset();
void MPU6050_I2C_MST_Reset();
void MPU6050_SIG_COND_Reset();

void MPU6050_FIFO_EnOrDi(uint8_t EnOrDi);

void MPU6050_I2C_MaterMode_EnOrDi(uint8_t EnOrDi);

void MPU6050_SleepMode_EnOrDi(uint8_t EnOrDi);
void MPU6050_TempSenor_EnOrDi(uint8_t EnOrDi);
void MPU6050_CycleMode_config(uint8_t temp);//TODO

void Read_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t *data );
void Write_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t data);
void Read_Multiple_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t *data , uint8_t Len );
void Write_Multiple_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t *data , uint8_t len);
//TODO
// To make slave function to set slave address and to read slave address
//function for slave swap
//slave group
//slave interrupt enable
// I2C_MST_DELAY_CTRL register
//read FIFO

/*for standby
 * for FIFO count
 * DMP
 */

#endif /* MPU6050_DRIVER_H_ */
