/*
 * MPU6050_Driver.c
 *
 *  Created on: 28-Jul-2021
 *  Last revision : 06/08/21
 *      Author: kamal chopra
 */


/*=================================================================================================================
 * 										Design flow of this driver
 *
 ******************************************************************************************************************
 * ==> About I2C communication sequence for sending/Receiving data to/from MPU:6050
 * (according to this we have to code for sending/receiving data to/from system processor(MCU) through I2C to MPU )
 *
 *	[Taken from the MPU-6000 and MPU-6050 product specification Revision 3.4 ]
 *
 * Single Byte Write Sequence
 * Start->MPU'sADDR with W ->wait for MPU ACK ->ADDR of register we want to write -> wait for ACk from MPU ->data to MPU
 * -> wait for ACk from MPU -> STOP
 *
 * Burst Write Sequence
 * Start->MPU'sADDR with W ->wait for MPU ACK ->ADDR of register we want to write ->wait for ACk from MPU ->data to send
 * -> wait for ACk from MPU ->data to send  -> wait for ACk from MPU ->............->STOP
 *
 * ## in this case MPU60x0 automatically increments the register address and load the data to the appropriate register.
 *
 * Single Byte Read Sequence
 * Start->MPU'sADDR with W ->wait for MPU ACK ->ADDR of register we want to write -> wait for ACk from MPU -> STOP
 * ->Start->MPU's ADDR with R-> wait for ACk from MPU -> data from MPU->wait for MCU ACK->data->wait for MCU ACK
 * ->...................->wait for MCU NACK->STOP
 *
 * (MCU - micro controller MPU - our sensor )
 *
 **********************************************************************************************************************
 **********************************************************************************************************************
 *
 * ==>> ABOUT DRIVER FUNCTIONS
 *
 * we know As every time we have to communicate  with our sensor we have to send the MPU address to select our sensor (as per the
 * I2C rules ) send the bit R/W according to need then send the register address we want to read/write then send/receive the
 * data.
 *
 * We have define functions  according to the bit they operate on or the register they operate on or for the specific
 * function like enable self test (to enable self test on all parameters ) , enable self test of x axis of gyro
 * (to enable self test on particular axis ) , to conif gyro to set all the parameters need to sent the gyro etc...
 *
 * no matter what the function is we need MPU address and the register address on which the function will work to fulfill
 * its job , so by asking these things from user every time is waste of processor cycle as it has to pass these values from function
 * to function also the user have to use the register manual side by side as he/she have to send the register address with the
 * functions so use of the driver useless as user is refereeing to manual
 *
 * so to code the MPU60x0 without the manual just by our driver have incorporate the register address in the functions
 * according to the job which that function has to perform. By this user can just use the functions of our driver code
 * without using reference manual for simple application .
 *
 *
 * ******************************************************************************************************************
 *
 *
 *
 * NOTE1:As our sensor MPU6050/MPU60000 is connected through  I2C interface to our micro controller  So to
 * 		 Read some data from the sensor or write data to the sensor or configure its properties/functions we need to
 * 		 send data through I2c interface so all the functions of this driver call(uses) I2C pheripheral of our MCU
 * 		 to Commuate with the sensor .
 * 		 To make driver generic so it can be used with all the MCU which support I2C interface all functions  call
 * 		 a functions calles :
 * 		 these functions inturn calls the MCU's I2C peripheral function to communicate this have increased layer of code
 * 		 as now to configur the senor main calls -> MPU's function -> I2C function of MPU -> MCU's I2c functions
 * 		 but the major advantage of this is that if we have to use the driver with any other MCU we just have to change
 * 		 a couple of functions with the I2C functions of that MCU.
 *
 * NOTE2:Sensor MPU6050/MPU60000 have a auxiliary I2C bus interface because of which more sensors can be connected
 * 		 with this sensor this MPU will act as the master for those sensor it will collect their data in its register
 * 		 and uses its DMP on that data if needed then interrupt the processor and send all the data in burst mode to
 * 		 the system processor this saves the processoring time of the system processor as now the system processor
 * 		 is only connect to one slave i.e our MPU and read all the data in a single go from its register .
 *
 * 		 The functions : MPU6050_I2C are for configuring the auxiliary bus of our MPU
 *
 *
 * ******************************************************************************************************************
 *
 * ================================================================================================================
 * ================================================================================================================
 */




/*===================================================================================================================
 *						Outline for Writing Application Program for this driver (MPU6050)
 *********************************************************************************************************************
 * V.IMP : NOTE : When we start the our MPU60x0 it is in sleep mode so in any case the first function call should
 * 		  be disable Sleep mode i.e   void MPU6050_Sleep_disable(); ( before this void MPU6050_Set_ADDR(uint8_t address) )
 *
 *
 *1) Writing program for self testing Ref @ProcedureForSelfTest for details :
 *	 I) Complete self test
 *	 	a) call void MPU6050_SelfTest_Config();
 *	 	b)
 *
 *
 *
 *2) Writing program for reading the data from Accel , Gyro and temp sensor
 *
 *
 *3)Writing program for reading data from  Accel sensor only
 *
 *
 *4)Writing program for reading data from Gyro  sensor Only
 *
 *
 *5)Writing program for reading data from slave (can be used with any of the above 4)
 *
 *
 *NOTE 2 : FOR interrupt from pin INT of the sensor
 *
 *
 *
 *********************************************************************************************************************
 *====================================================================================================================
 */






/* 	Eg. of the sequence in which the functions have to called for some basic applications of the sensor
 *
 *
 *
 * Note : it is just for the idea , we can follow any order as per our need or the application demand
 */

#include "MPU6050_Driver.h"
#include <math.h>

 /*working on it
  *  for parameters refer @address
  */
uint8_t mpuADDR = 0 ; // to store the MPU60x0 address which we take from the user
 /*****************************************************************
  * @fn			- void MPU6050_Set_ADDR(uint8_t address);
  *
  * @brief		- To send the address of the  MPU6050
  *
  * @param[address]	- address of the MPU60x0
  * @param[in]	- N/A
  *
  * @return		- None
  *
  * @Note		- function is called to store the address of our MPU60x0
  * 				  PWR_MGMT_1 register (0x6B).
  *				- It should be the first function to be called by the programmer
  *				  if he is using this MPU60x0 driver because it supplies address
  *				  of the MPU to the MCU which will be needed
  *				  in communication through our I2C.
  *				  Because only after calling this we could communicate the the MPU
  *
  *				- Say if the design has 2 MPU60x0 then programmer has to call
  * 			  this function and supply the address of the first MPU after which call
  * 			  the other functions needed to configure that MPU then
  * 			  he again have to call this function and supply the address of the second MPU60x0
  * 			  then call the functions needed to cofigure that MPU according to the application
  *
  *****************************************************************/
void MPU6050_Set_ADDR(uint8_t address)
{
	mpuADDR = address;
}

/*****************************************************************
 * @fn			- void MPU6050_Sleep_disable()
 *
 * @brief		- To disable the sleep mode
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- It should be the second function to be called if this driver
 * 			 	  is been used. Because as per the datasheet the intinally our
 * 			 	  MPU6050 is in sleep mode so we have weak it up before communicating
 * 			 	  with it .
 *
 * 			 	- In this  MPU6050_SLEEP_EN_BIT of register MPU6050_PWR_MGMT_1_REG_ADDR
 * 			 	 is reseted.
 * 			 	 i.e
 * 			 	 when MPU6050_SLEEP_EN_BIT = 1 : puts the MPU-60X0 into sleep mode.
 * 			 	 when MPU6050_SLEEP_EN_BIT = 0 : Not is sleep mode
 *
 *
 *****************************************************************/
void MPU6050_Sleep_disable()
{
	uint8_t ToSend = 0 ;
	ToSend &= ~(1 << MPU6050_SLEEP_EN_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR , ToSend);
}

/*
 * MPU6050 : Device Related functions
 */

/*****************************************************************
 * @fn			- MPU6050_Reset()
 *
 * @brief		- To reset our MPU6050
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we set the DEVICE_RESET bit of the
 * 				  PWR_MGMT_1 register (0x6B).
 *
 * 				  DEVICE_RESET bit :
 * 				  When set to 1, this bit resets all internal
 * 				  registers to their default values.
 *				  The bit automatically clears to 0 once the reset is done.
 *
 *****************************************************************/
void MPU6050_Reset()
{
	uint8_t data = 0 ;
	data |= ( 1 << MPU6050_DEVICE_RESET_BIT) ;

	Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR, data);

}

/*****************************************************************
 * @fn			- MPU6050_DLPF_Config
 *
 * @brief		- This function is to set the DLPF values of our
 * 				  MPU60x0
 *
 * @param[DLPFVal]	- value we want to config
 * @param[in]		- N/A
 *
 * @return		- None
 *
 * @Note		- DLPF : Digital Low Pass Filter
 * 				  the signals of accelerometer and
 * 				  gyroscope are filtered according to the value of DLPF_CFG
 *
 * 				  In this function we config the DLPF_CFG[2:0] bit of the
 * 				  CONFIG register (0x1A).
 *
 * 				  DLPF_CFG[2:0] bit :
 * 				  3-bit unsigned value. Configures the DLPF setting
 * 				  DLPF can be any value from 0 to 6
 * 				  each of this value has a predefined bandwidth and delay
 * 				  for the accelrometer and the gyroscope senor
 * 				  for the values refer table on page 13 of register manual
 *
 * @Note		- DLFP value affect the output rate of the gyro and accel sensor
 * 				  Gyroscope output rate = 8Khz  when DLFP = 0
 * 				  and 1kHz when DLFP is in between 1 - 6
 * 				  accelerometer output rate remains 1kHz no matter what is the val of
 * 				  DLFP
 *
 *
 *****************************************************************/
void MPU6050_DLPF_Config(uint8_t DLPFVal)
{
	uint8_t data = 0 ;
		data |= (DLPFVal <<  MPU6050_CFG_DLPF_COFIG_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_CONFIG_REG_ADDR, data);
}

/*****************************************************************
 * @fn			- MPU6050_FSYNC_Config
 *
 * @brief		- This function we config the external Frame
 * 				  Synchronization (FSYNC) pin sampling
 * 				  i.e our MPU has a pin called FSYNC
 *
 *
 * @param[FSYNCVal]	- value we want to configured
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- FSYNC : external Frame Synchronization pin sampling
 * 				  IN THIS WE CONFIG THE EXT_SYNC_SET[2:0] bit of
 * 				  CONFIG register(0x1A) this bit can take any value
 * 				  in between 0 to 7 .
 * 				  values is choose according to the signal we want to
 * 				  connect on the FSYC pin ( refer table on Pg 13 of register manaul)
 *
 * @Note		- An external signal connected to the FSYNC pin can
 * 				  be sampled by configuring EXT_SYNC_SET.
				  Signal changes to the FSYNC pin are latched so that
				  short strobes may be captured. The latched FSYNC signal
				  will be sampled at the Sampling Rate of the MPU which we have
				  configured.
				  After sampling, the latch will reset to the current FSYNC
				  signal state. The sampled value will be reported in
				  place of the least significant bit in a sensor data
				  register determined by the value of EXT_SYNC_SET
				  according to the following table.
 *
 *****************************************************************/
void MPU6050_FSYNC_Config(uint8_t FSYNCVal)
{
	uint8_t data = 0 ;
		data |=(FSYNCVal << MPU6050_CFG_EXT_SYNC_SET_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_CONFIG_REG_ADDR, data);
}

/*****************************************************************
 * @fn			- MPU6050_DLPF_GetVal()
 *
 * @brief		- This function is to read the val of the DLPF
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- DLPF VAL
 *
 * @Note		- DLPF : Digital Low Pass Filter
 * 				  this function is used to read the the DLPF_CFG[2:0]
 * 				  bit of the CONFIG register (0x1A).
 * 				  this is needed if we need DLPF val for our calulations
 * 				  in the application
 *
 *****************************************************************/
uint8_t MPU6050_DLPF_GetVal()
{
	uint8_t data = 0;
	// reading the CONFIG register
	Read_MPU6050_I2C(mpuADDR, MPU6050_CONFIG_REG_ADDR , &data);
	// shift the register value to get the Bit val needed
	data &= (7 << MPU6050_CFG_DLPF_COFIG_BIT );
	  return data ;
}

/*****************************************************************
 * @fn			- MPU6050_FSYNC_GetVal
 *
 * @brief		- This function is to read the val of the FSYNC
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- FSYNC Val
 *
 * @Note		- None
 *
 *****************************************************************/
uint8_t MPU6050_FSYNC_GetVal()
{
	uint8_t data = 0;
	// reading the CONFIG register
	Read_MPU6050_I2C(mpuADDR, MPU6050_CONFIG_REG_ADDR , &data);
	// shift the register value to get the Bit val needed
	data &= (7 << MPU6050_CFG_EXT_SYNC_SET_BIT );
	  return data ;
}

/*****************************************************************
 * @fn			- MPU6050_GetMPU_ADDR()
 *
 * @brief		- This function is to get identity of the device.
 *
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- return the address of the MPU60x0
 * 				  it returns as 6 bit values
 *
 * @Note		- In this function we read the WHO_AM_I register(0x75)
 * 				  of our MPU60x0 to get its address which is needed for
 * 				  I2C communication
 *
 *@Note			- This register is used to verify the identity of the device.
 *				  The contents of WHO_AM_I are the upper 6 bits of the
 *			   	  MPU-60X0’s 7-bit I2C address. The least significant bit
 *		 		  of the MPU-60X0’s I2C address is determined by the value
 *			  	  of the AD0 pin. The value of the AD0 pin is not reflected
 *			   	  in this register.
 *				  The default value of the register is 0x68.
 *
 *				- Need for getting the identy:
 *			  	  sometimes we may need to crosscheck that the data
 *			  	  we are reading to writing to is correct device if their
 *			  	  are multiple sensors like this in the application.
 *
 *****************************************************************/
uint8_t MPU6050_GetMPU_ADDR()
{
	uint8_t data ;
	 Read_MPU6050_I2C(mpuADDR, MPU6050_WHO_AM_I_REG_ADDR, &data);
	 data |= (data >> 1 );
	 //returning the 6 bits of the address then to this val
	 //add a LSB bit base on A0 pin to get the proper 7 bit address
	return data;
}


/**************************************************************************************************************
 *  		FUNCTIONS RELATED TO SELF TEST FEATURE OF MPU60X0
 *
 * method/ procedure to conduct the self test { @ProcedureForSelfTest } :
 *
 * self-test permits users to test the mechanical and electrical portions of the gyroscope / accelerometer.
 *
 * SelfTest Response(STR) = Gyroscope output with self test Enabled - Gyroscope output with self test disabled
 *
 * 	Change from factory trim of the self test response(%) = (STR -FT)/FT
 * 		FT = Factory trim value of the self test response
 *
 *	To find FT value :
 * @FT_FORMULA
 *	Gyroscope :
 *
 *	FT[Xg] = 25*131*1.046^(XG_TEST - 1 ) 				if XG_TEST != 0
 *	FT[Xg] = 0							 				if XG_TEST  = 0
 *
 * 	FT[Yg] = - 25*131*1.046^(YG_TEST - 1 ) 				if YG_TEST != 0
 *	FT[Yg] = 0							 				if YG_TEST  = 0
 *
 *	FT[Zg] = 25*131*1.046^(ZG_TEST - 1 ) 				if ZG_TEST != 0
 *	FT[Zg] = 0							 				if ZG_TEST  = 0
 *
 *	Acceleromter :
 *
 *	FT[Xa] = 4096*0.34*(0.92/0.34)^((XA_TEST-1)/2^5 -2) if XA_TEST != 0
 *	FT[Xa] = 0							 				if XA_TEST  = 0
 *
 * 	FT[Ya] = 4096*0.34*(0.92/0.34)^((YA_TEST-1)/2^5 -2) if YA_TEST != 0
 *	FT[Ya] = 0							 				if YA_TEST  = 0
 *
 *	FT[Za] = 4096*0.34*(0.92/0.34)^((ZA_TEST-1)/2^5 -2) if ZA_TEST != 0
 *	FT[Za] = 0							 				if ZA_TEST  = 0
 *
 *
 *		Change from factory trim of the self test response(%) = (STR -FT)/FT
 *
 *This change from factory trim of the self-test response must be within the limits provided in the MPU-6000/MPU-6050
 *Product Specification document for the part to pass self-test. Otherwise, the part is deemed to have failed self-test.
 *
 *STEPS : { @StepsOfSelfTest }
 *	1) ENABLE the self test for the required axis
 *	2) Read the self test register value for the required axis ( to get XA_test , XG_TEST etc... values)
 *	3) calculate the self trim value
 *	4) read the data register of the required axis for the  required sensor
 *	5) turn off the self test mode for the axis we are calculating
 *	6) Again read the data register we read earlier
 *	7) Calculate  the self test response value ( from the formula )
 *	8) find the change from factory trim of the self test response(%) from the formula
 *	9) check if this % is in the range of the factory value or not if not then part failed self test
 *
 *
 * *@V.IMP		-  IN SELF TEST RELATED FUNCTIONS  WE PERFORM FLOATING POINT CALCULATIONS SO ENABLE THE FPU OF THE MCU
 *				   OR USE THE SOFTWARE BASED FPU .
 **************************************************************************************************************/

/*****************************************************************
 * @fn			- MPU6050_SelfTest_Config()
 *
 * @brief		- this function is for self testing the all the sensors
 * 				  for all the axis
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- it returns a 8 bit value
 * 				  In which lower 3 bits are for the Z ,Y X axis of the accelerometer
 * 				  and the next 3 bits (that is from bit 3 to 5 ) is for the X ,y , Z
 * 				  axis of the gyroscope.
 * 				  if the given Bit is set then it means that sensor has passed the
 * 				  self test for that axis
 * 				  if the given Bit is in reset state then it means the
 * 				  sensor of the axis has failed the self test.
 *
 * 				  Significance of Bits of Return value
 *
 * 				  BIT 0 : Accel Z Axis
 * 				  BIT 1 : Accel Y Axis
 * 				  BIT 2 : Accel X Axis
 * 				  BIT 3 : Gyro  Z Axis
 * 				  BIT 4 : Gyro  Y Axis
 * 				  BIT 5 : Gyro	X Axis
 * 				  BIT 6 : don't care
 * 				  BIT 7 : don't care
 *
 * 				  So we have to check each bit individually to see the self test response of each axis
 *
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 * 				- In this we perform self test on the all the axis of the accelerometer
 * 					and all the axis of the gyroscope.
 * 					also perform all the calculations and resturn the result that is
 * 					which axis of the accel and gyro have passed the test and which have failed
 *
 *@V.IMP		-  IN THIS WE PERFORM FLOATING POINT CALCULATIONS SO ENABLE THE FPU OF THE MCU
 *				   OR USE THE SOFTWARE BASED FPU .
 *****************************************************************/
uint8_t MPU6050_SelfTest_Config()
{

	//1. Enable self test bits of gyro and accel
	uint8_t temp = 0 ;
	temp = (0xE0 ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);

	for(uint8_t i = 0 ; i <30 ; ++i);
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);
	for(uint8_t i = 0 ; i <30 ; ++i);
	//2. read read self test data register

	uint8_t data[4] = {0,0,0,0} ;

	uint8_t accel_XTestdata = 0 , accel_YTestdata = 0 , accel_ZTestdata = 0;
	uint8_t gyro_XTestdata = 0 , gyro_YTestdata = 0 , gyro_ZTestdata = 0;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_X_REG_ADDR , data, 4);
	for(uint8_t i = 0 ; i <30 ; i++);
	accel_XTestdata |= ( ( data[0]  & 0xE0 ) >> 3 );
	gyro_XTestdata  |=  ( ( data[0]  & 0x1F ) >> 0 );
	accel_XTestdata |= ( ( data[3]  & 0x30 ) >> 4 );

	accel_YTestdata |= ( ( data[1]  & 0xE0 ) >> 3 );
	gyro_YTestdata  |= ( ( data[1]  & 0x1F ) >> 0 );
	accel_YTestdata |= ( ( data[3]  & 0x0C ) >> 2 );

	accel_ZTestdata |= ( ( data[2]  & 0xE0 ) >> 3 );
	gyro_ZTestdata  |= ( ( data[2]  & 0x1F ) >> 0 );
	accel_ZTestdata |= ( ( data[3]  & 0x03 ) >> 0 );



	//3. read  data register
	uint8_t accel_data[6] , gyro_data[6] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_XOUT_H_REG_ADDR, accel_data, 6);
	for(uint8_t i = 0 ; i <30 ; i++);
	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_XOUT_H_REG_ADDR, gyro_data, 6);
	for(uint8_t i = 0 ; i <30 ; i++);
	uint16_t accel_Xdata , accel_Ydata , accel_Zdata ;
	uint16_t gyro_Xdata , gyro_Ydata , gyro_Zdata;

	accel_Xdata = ((uint16_t)accel_data[0] << 8);
	accel_Xdata |= accel_data[1] ;

	accel_Ydata = ((uint16_t)accel_data[2] << 8);
	accel_Ydata |= accel_data[3] ;

	accel_Zdata = ((uint16_t)accel_data[4] << 8);
	accel_Zdata |= accel_data[5] ;

	gyro_Xdata = ((uint16_t)gyro_data[0] << 8);
	gyro_Xdata |= gyro_data[1] ;

	gyro_Ydata = ((uint16_t)gyro_data[2] << 8);
	gyro_Ydata |= gyro_data[3] ;

	gyro_Zdata = ((uint16_t)gyro_data[4] << 8);
	gyro_Zdata |= gyro_data[5] ;

	//4. turn off the self test of the sensors

	uint8_t temp2 ;
	temp2 = (0x1F ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp2);
	for(uint8_t i = 0 ; i <30 ; i++);
	temp2 = (0x1F ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp2);
	for(uint8_t i = 0 ; i <30 ; i++);
	//5. read the output data register of sensor when self test is disabled

	uint8_t accel_data2[6] , gyro_data2[6] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_XOUT_H_REG_ADDR, accel_data2, 6);
	for(uint8_t i = 0 ; i <30 ; i++);
	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_XOUT_H_REG_ADDR, gyro_data2 , 6);
	for(uint8_t i = 0 ; i <30 ; i++);
	uint16_t accel_Xdata2 , accel_Ydata2 , accel_Zdata2 ;
	uint16_t gyro_Xdata2 , gyro_Ydata2 , gyro_Zdata2 ;

	accel_Xdata2  = ((uint16_t)accel_data2[0] << 8);
	accel_Xdata2 |= accel_data2[1] ;

	accel_Ydata2  = ((uint16_t)accel_data2[2] << 8);
	accel_Ydata2 |= accel_data2[3] ;

	accel_Zdata2  = ((uint16_t)accel_data2[4] << 8);
	accel_Zdata2 |= accel_data2[5] ;

	gyro_Xdata2  = ((uint16_t)gyro_data2[0] << 8);
	gyro_Xdata2 |= gyro_data2[1] ;

	gyro_Ydata2  = ((uint16_t)gyro_data2[2] << 8);
	gyro_Ydata2 |= gyro_data2[3] ;

	gyro_Zdata2 = ((uint16_t)gyro_data2[4] << 8);
	gyro_Zdata2 |= gyro_data2[5] ;

	//6. Find STR value

	//SelfTest Response(STR) = Gyroscope output with self test Enabled - Gyroscope output with self test disabled

	uint8_t accelX_STR , accelY_STR , accelZ_STR ;
	uint8_t gyroX_STR , gyroY_STR , gyroZ_STR ;

	accelX_STR = accel_Xdata - accel_Xdata2 ;
	accelY_STR = accel_Ydata - accel_Ydata2 ;
	accelZ_STR = accel_Zdata - accel_Zdata2 ;

	gyroX_STR = gyro_Xdata - gyro_Xdata2 ;
	gyroY_STR = gyro_Ydata - gyro_Ydata2 ;
	gyroZ_STR = gyro_Zdata - gyro_Zdata2 ;


	//7. calculate the FT value ( using the formula from dataSheet and
	//			the self test data register value ) @FT_FORMULA

	float accelX_FT_val , accelY_FT_val ,accelZ_FT_val ;
	if ( accel_XTestdata!= 0 )
	{
		//FT[Xg] = 25*131*1.046^(XG_TEST - 1 )
		accelX_FT_val = pow (2,5); // for testing
		accelX_FT_val = pow(1 , ( accel_XTestdata - 1)) ;// for testing
		accelX_FT_val = pow(1.046 , ( accel_XTestdata - 1)) ;
	//	accelX_FT_val = 25*131*accelX_FT_val;// for testing
		accelX_FT_val = pow(3425 , ( accel_XTestdata - 1)) ; // testing
		accelX_FT_val = pow(3425.65 , ( accel_XTestdata - 1)) ;

	}else
	{
		//FT[Xg] = 0

		accelX_FT_val = 0 ;
	}

	if (accel_YTestdata != 0)
	{
		//FT[Yg] = - 25*131*1.046^(YG_TEST - 1 )
		accelY_FT_val = - ( pow( 3425.65 , ( accel_YTestdata - 1) ) ) ;

	}else
	{
		//FT[Yg] = 0

		accelY_FT_val = 0 ;
	}

	if (accel_ZTestdata != 0)
	{
		//FT[Zg] = 25*131*1.046^(ZG_TEST - 1 )
		accelZ_FT_val = ( pow( 3425.65 , ( accel_ZTestdata - 1) ) ) ;

	}else
	{
		//FT[Zg] = 0

		accelZ_FT_val = 0 ;
	}


//==============================================
	float gyroX_FT_val , gyroY_FT_val , gyroZ_FT_val ;

	if ( gyro_XTestdata!= 0 )
	{
		//FT[Xg] = 25*131*1.046^(XG_TEST - 1 )
		gyroX_FT_val = pow(3425.65 , (gyro_XTestdata - 1)) ;

	}else
	{
		//FT[Xg] = 0

		gyroX_FT_val = 0 ;
	}

	if (gyro_YTestdata != 0)
	{
		//FT[Yg] = - 25*131*1.046^(YG_TEST - 1 )
		gyroY_FT_val = - ( pow( 3425.65 , (gyro_YTestdata - 1) ) ) ;

	}else
	{
		//FT[Yg] = 0

		gyroY_FT_val = 0 ;
	}

	if (gyro_ZTestdata != 0)
	{
		//FT[Zg] = 25*131*1.046^(ZG_TEST - 1 )
		gyroZ_FT_val = ( pow( 3425.65 , ( gyro_ZTestdata - 1) ) ) ;

	}else
	{
		//FT[Zg] = 0

		gyroZ_FT_val = 0 ;
	}




	//8. using the STR value and FT value find the change from factory trim value percentage

	//Compare the above calculated percentage with the factory given values

	//Change from factory trim of the self test response(%) = (STR -FT)/FT

	float gyroX_percent , gyroY_percent , gyroZ_percent;
	float accelX_percent , accelY_percent , accelZ_percent;

	accelX_percent = ( (accelX_STR - accelX_FT_val) / accelX_FT_val );

	accelY_percent = ( (accelY_STR - accelY_FT_val  ) / accelY_FT_val );

	accelZ_percent = ( (accelZ_STR -  accelZ_FT_val ) / accelZ_FT_val );

	gyroX_percent = ( ( gyroX_STR - gyroX_FT_val) /gyroX_FT_val);

	gyroY_percent = (( gyroY_STR - gyroY_FT_val ) / gyroY_FT_val );

	gyroZ_percent = (( gyroZ_STR - gyroZ_FT_val) / gyroZ_FT_val );
	uint8_t result = 0 ;

	if( ( gyroX_percent < 14) &&  (gyroX_percent > -14 ) )
		result |= (1 << 5);
	if( ( gyroY_percent < 14) &&  (gyroY_percent > -14 ) )
		result |= (1 << 4);
	if( ( gyroZ_percent < 14) &&  (gyroZ_percent > -14 ) )
		result |= (1 << 3);
	if( ( accelX_percent < 14) && (accelX_percent > -14 ) )
		result |= (1 << 2);
	if( ( accelY_percent < 14) && (accelY_percent > -14 ) )
		result |= (1 << 1);
	if( ( accelZ_percent < 14) && (accelZ_percent > -14 ) )
		result |= (1 << 0 );

	return result ;
}



//function to self test individual accel axis

/*****************************************************************
 * @fn			- uint8_t MPU6050_XGyro_SelfTest()
 *
 * @brief		- This function is used to perform self test on the
 * 			  	  X axis of the gyro scope
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- the result of the self test
 * 				  if it returns 0 : the self test has failed
 * 				  if it returns 1 : the self test has passed
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 *****************************************************************/
uint8_t MPU6050_XGyro_SelfTest()
{
	//1) enable the self test bit
	uint8_t temp = 0 ;
	temp |= (1 << MPU6050_GYRO_XG_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);

	//2) read  self test data register

	uint8_t data ;
	uint8_t gyro_XTestdata = 0 ;

	Read_MPU6050_I2C(mpuADDR,  MPU6050_SELF_TEST_X_REG_ADDR , &data);

	gyro_XTestdata  |=  ( ( data  & 0x1F ) >> 0 );

	//3) read the output data register  accel/gyro

	uint8_t  gyro_data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_XOUT_H_REG_ADDR, gyro_data, 2);

	uint16_t gyro_Xdata ;

	gyro_Xdata = ((uint16_t)gyro_data[0] << 8);
	gyro_Xdata |= gyro_data[1] ;

	//4) disable the self test
	temp &= ~(1 << MPU6050_GYRO_XG_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);


	//5) read the output data register accel/gyro

	uint8_t  gyro_data2[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_XOUT_H_REG_ADDR, gyro_data2 , 2);

	uint16_t gyro_Xdata2  ;

	gyro_Xdata2  = ((uint16_t)gyro_data2[0] << 8);
	gyro_Xdata2 |= gyro_data2[1] ;

	//6) Find STR value

	uint8_t gyroX_STR  ;

	gyroX_STR = gyro_Xdata - gyro_Xdata2 ;

	//7) calculate the FT value ( using the formula from dataSheet and
		//			the self test data register value ) @FT_FORMULA


	double gyroX_FT_val  ;
	if ( gyro_XTestdata!= 0 )
	{
		//FT[Xg] = 25*131*1.046^(XG_TEST - 1 )
		gyroX_FT_val = pow(3425.65 , (gyro_XTestdata - 1)) ;

	}else
	{
		//FT[Xg] = 0

		gyroX_FT_val = 0 ;
	}

	//8) using the STR value and FT value find the change from factory trim value percentage
	//Compare the above calculated percentage with the factory given values
	//Change from factory trim of the self test response(%) = (STR -FT)/FT

	double gyroX_percent ;


	gyroX_percent = ( ( gyroX_STR - gyroX_FT_val) /gyroX_FT_val);
	if( ( gyroX_percent < 14) &&  (gyroX_percent > -14 ) )
		return 1;

	return 0;

}

/*****************************************************************
 * @fn			- uint8_t MPU6050_YGyro_SelfTest()
 *
 * @brief		- This function is used to perform self test on the
 * 			  	  y axis of the gyro scope
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- the result of the self test
 * 				  if it returns 0 : the self test has failed
 * 				  if it returns 1 : the self test has passed
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 *****************************************************************/
uint8_t MPU6050_YGyro_SelfTest()
{
	//1) enable the self test bit
	uint8_t temp = 0 ;
	temp |= (1 << MPU6050_GYRO_YG_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);

	//2) read read self test data register

	uint8_t data ;
	uint8_t  gyro_YTestdata = 0 ;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Y_REG_ADDR, &data);
	gyro_YTestdata  |= ( ( data  & 0x1F ) >> 0 );

	//3) read the output data register  accel/gyro

	uint8_t  gyro_data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_YOUT_H_REG_ADDR, gyro_data, 2);

	uint16_t gyro_Ydata ;

	gyro_Ydata = ((uint16_t)gyro_data[0] << 8);
	gyro_Ydata |= gyro_data[1] ;


	//4) disable the self test
	temp &= ~(1 << MPU6050_GYRO_YG_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);


	//5) read the output data register accel/gyro

	uint8_t  gyro_data2[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_YOUT_H_REG_ADDR, gyro_data2 , 2);

	uint16_t gyro_Ydata2 ;

	gyro_Ydata2  = ((uint16_t)gyro_data2[0] << 8);
	gyro_Ydata2 |= gyro_data2[1] ;

	//6) Find STR value

	uint8_t gyroY_STR  ;
	gyroY_STR = gyro_Ydata - gyro_Ydata2 ;


	//7) calculate the FT value ( using the formula from dataSheet and
		//			the self test data register value ) @FT_FORMULA

	double  gyroY_FT_val ;
	if (gyro_YTestdata != 0)
	{
		//FT[Yg] = - 25*131*1.046^(YG_TEST - 1 )
		gyroY_FT_val = - ( pow( 3425.65 , (gyro_YTestdata - 1) ) ) ;

	}else
	{
		//FT[Yg] = 0

		gyroY_FT_val = 0 ;
	}

	//8) using the STR value and FT value find the change from factory trim value percentage

	//Compare the above calculated percentage with the factory given values

	//Change from factory trim of the self test response(%) = (STR -FT)/FT
	double  gyroY_percent ;
	gyroY_percent = (( gyroY_STR - gyroY_FT_val ) / gyroY_FT_val );

	if( ( gyroY_percent < 14) &&  (gyroY_percent > -14 ) )
		return 1 ;

	return 0 ;

}

/*****************************************************************
 * @fn			- uint8_t MPU6050_ZGyro_SelfTest()
 *
 * @brief		- This function is used to perform self test on the
 * 			  	  Z axis of the gyro scope
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- the result of the self test
 * 				  if it returns 0 : the self test has failed
 * 				  if it returns 1 : the self test has passed
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 *****************************************************************/
uint8_t MPU6050_ZGyro_SelfTest()
{
	//1) enable the self test bit
	uint8_t temp = 0 ;
	temp |= (1 << MPU6050_GYRO_ZG_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);

	//2) read read self test data register

	uint8_t data ;
	uint8_t  gyro_ZTestdata = 0;
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Z_REG_ADDR, &data);
	gyro_ZTestdata  |= ( ( data  & 0x1F ) >> 0 );

	//3) read the output data register  accel/gyro

	uint8_t  gyro_data[2] ;
	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_ZOUT_H_REG_ADDR , gyro_data, 2);

	uint16_t  gyro_Zdata;

	gyro_Zdata = ((uint16_t)gyro_data[0] << 8);
	gyro_Zdata |= gyro_data[1] ;

	//4) disable the self test
	temp &= ~(1 << MPU6050_GYRO_ZG_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, temp);


	//5) read the output data register accel/gyro

	uint8_t  gyro_data2[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_ZOUT_H_REG_ADDR, gyro_data2 , 2);

	uint16_t  gyro_Zdata2 ;

	gyro_Zdata2 = ((uint16_t)gyro_data2[0] << 8);
	gyro_Zdata2 |= gyro_data2[1] ;

	//6) Find STR value

	uint8_t gyroZ_STR ;
	gyroZ_STR = gyro_Zdata - gyro_Zdata2 ;

	//7) calculate the FT value ( using the formula from dataSheet and
		//			the self test data register value ) @FT_FORMULA

	double  gyroZ_FT_val ;
	if (gyro_ZTestdata != 0)
	{
		//FT[Zg] = 25*131*1.046^(ZG_TEST - 1 )
		gyroZ_FT_val = ( pow( 3425.65 , ( gyro_ZTestdata - 1) ) ) ;

	}else
	{
		//FT[Zg] = 0

		gyroZ_FT_val = 0 ;
	}

	//8) using the STR value and FT value find the change from factory trim value percentage

	//Compare the above calculated percentage with the factory given values

	//Change from factory trim of the self test response(%) = (STR -FT)/FT
	double  gyroZ_percent;
	gyroZ_percent = (( gyroZ_STR - gyroZ_FT_val) / gyroZ_FT_val );

	if( ( gyroZ_percent < 14) &&  (gyroZ_percent > -14 ) )
		return 1;

	return 0 ;
}

//function to self test individual gyro axis

/*****************************************************************
 * @fn			- uint8_t MPU6050_XAccel_SelfTest()
 *
 * @brief		- This function is used to perform self test on the
 * 			  	  X axis of the ACCEL scope
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- the result of the self test
 * 				  if it returns 0 : the self test has failed
 * 				  if it returns 1 : the self test has passed
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 *****************************************************************/
uint8_t MPU6050_XAccel_SelfTest()
{
	//1) enable the self test bit

	uint8_t temp = 0 ;
	temp |= (1 << MPU6050_ACCEL_XA_ST_BIT  ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);

	//2) read read self test data register
	uint8_t data[2] ;
	uint8_t accel_XTestdata = 0 ;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_X_REG_ADDR, &data[0]);
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_A_REG_ADDR, &data[1]);

	accel_XTestdata |= ( ( data[0]  & 0xE0 ) >> 3 );
	accel_XTestdata |= ( ( data[1]  & 0x30 ) >> 4 );

	//3) read the output data register  accel/gyro

	uint8_t accel_data[2]  ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_XOUT_H_REG_ADDR, accel_data, 2);

	uint16_t accel_Xdata ;

	accel_Xdata = ((uint16_t)accel_data[0] << 8);
	accel_Xdata |= accel_data[1] ;

	//4) disable the self test
	temp &= ~(1 << MPU6050_ACCEL_XA_ST_BIT  ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);


	//5) read the output data register accel/gyro

	uint8_t accel_data2[2]  ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_XOUT_H_REG_ADDR, accel_data2, 2);

	uint16_t accel_Xdata2  ;

	accel_Xdata2  = ((uint16_t)accel_data2[0] << 8);
	accel_Xdata2 |= accel_data2[1] ;


	//6) Find STR value
	uint8_t accelX_STR  ;
	accelX_STR = accel_Xdata - accel_Xdata2 ;

	//7) calculate the FT value ( using the formula from dataSheet and
		//			the self test data register value ) @FT_FORMULA

	double accelX_FT_val  ;
	if ( accel_XTestdata!= 0 )
	{
		//FT[Xg] = 25*131*1.046^(XG_TEST - 1 )
		accelX_FT_val = pow(3425.65 , ( accel_XTestdata - 1)) ;

	}else
	{
		//FT[Xg] = 0

		accelX_FT_val = 0 ;
	}
	//8) using the STR value and FT value find the change from factory trim value percentage
	//Compare the above calculated percentage with the factory given values
	//Change from factory trim of the self test response(%) = (STR -FT)/FT


	double accelX_percent ;

	accelX_percent = ( (accelX_STR - accelX_FT_val) / accelX_FT_val );

	if( ( accelX_percent < 14) && (accelX_percent > -14 ) )
		return 1 ;

	return 0;
}

/*****************************************************************
 * @fn			- uint8_t MPU6050_YAccel_SelfTest()
 *
 * @brief		- This function is used to perform self test on the
 * 			  	  Y axis of the ACCEL scope
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- the result of the self test
 * 				  if it returns 0 : the self test has failed
 * 				  if it returns 1 : the self test has passed
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 *****************************************************************/
uint8_t MPU6050_YAccel_SelfTest()
{
	//1) enable the self test bit
	uint8_t temp = 0 ;
	temp |= (1 << MPU6050_ACCEL_YA_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);

	//2) read read self test data register
	uint8_t data[2] ;
	uint8_t  accel_YTestdata = 0 ;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Y_REG_ADDR, &data[0]);
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_A_REG_ADDR, &data[1]);

	accel_YTestdata |= ( ( data[0]  & 0xE0 ) >> 3 );
	accel_YTestdata |= ( ( data[1]  & 0x0C ) >> 2 );

	//3) read the output data register  accel/gyro

	uint8_t accel_data[2]   ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_YOUT_H_REG_ADDR, accel_data, 2);

	uint16_t  accel_Ydata  ;

	accel_Ydata = ((uint16_t)accel_data[0] << 8);
	accel_Ydata |= accel_data[1] ;

	//4) disable the self test
	temp &= ~(1 << MPU6050_ACCEL_YA_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);


	//5) read the output data register accel/gyro

	uint8_t accel_data2[2]  ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_YOUT_H_REG_ADDR, accel_data2, 2);

	uint16_t accel_Ydata2  ;

	accel_Ydata2  = ((uint16_t)accel_data2[0] << 8);
	accel_Ydata2 |= accel_data2[1] ;

	//6) Find STR value
	uint8_t  accelY_STR;
	accelY_STR = accel_Ydata - accel_Ydata2 ;

	//7) calculate the FT value ( using the formula from dataSheet and
		//			the self test data register value ) @FT_FORMULA

	double  accelY_FT_val  ;
	if (accel_YTestdata != 0)
	{
		//FT[Yg] = - 25*131*1.046^(YG_TEST - 1 )
		accelY_FT_val = - ( pow( 3425.65 , ( accel_YTestdata - 1) ) ) ;

	}else
	{
		//FT[Yg] = 0

		accelY_FT_val = 0 ;
	}
	//8) using the STR value and FT value find the change from factory trim value percentage
	//Compare the above calculated percentage with the factory given values
	//Change from factory trim of the self test response(%) = (STR -FT)/FT


	double  accelY_percent ;


	accelY_percent = ( (accelY_STR - accelY_FT_val  ) / accelY_FT_val );


	if( ( accelY_percent < 14) && (accelY_percent > -14 ) )
		return 1;

	return 0 ;
}

/*****************************************************************
 * @fn			- uint8_t MPU6050_ZAccel_SelfTest()
 *
 * @brief		- This function is used to perform self test on the
 * 			  	  Z axis of the ACCEL scope
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- the result of the self test
 * 				  if it returns 0 : the self test has failed
 * 				  if it returns 1 : the self test has passed
 *
 * @Note		- Refer @ProcedureForSelfTest
 * 					  	@StepsOfSelfTest for the steps followed
 *
 *****************************************************************/
uint8_t MPU6050_ZAccel_SelfTest()
{
	//1) enable the self test bit
	uint8_t temp = 0 ;
	temp |= (1 << MPU6050_ACCEL_ZA_ST_BIT ) ;

	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);

	//2) read read self test data register
	uint8_t data[2] ;

	uint8_t  accel_ZTestdata = 0;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Z_REG_ADDR, &data[0]);
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_A_REG_ADDR, &data[1]);

	accel_ZTestdata |= ( ( data[0]  & 0xE0 ) >> 3 );
	accel_ZTestdata |= ( ( data[1]  & 0x03 ) >> 0 );
	//3) read the output data register  accel/gyro

	uint8_t accel_data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_ZOUT_H_REG_ADDR, accel_data, 2);

	uint16_t accel_Zdata ;

	accel_Zdata = ((uint16_t)accel_data[0] << 8);
	accel_Zdata |= accel_data[1] ;

	//4) disable the self test

	temp &= ~(1 << MPU6050_ACCEL_ZA_ST_BIT ) ;
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR, temp);


	//5) read the output data register accel/gyro

	uint8_t accel_data2[2]  ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_ZOUT_H_REG_ADDR, accel_data2, 2);

	uint16_t  accel_Zdata2 ;

	accel_Zdata2  = ((uint16_t)accel_data2[0] << 8);
	accel_Zdata2 |= accel_data2[1] ;

	//6) Find STR value
	uint8_t  accelZ_STR ;
	accelZ_STR = accel_Zdata - accel_Zdata2 ;

	//7) calculate the FT value ( using the formula from dataSheet and
		//			the self test data register value ) @FT_FORMULA

	double accelZ_FT_val ;

	if (accel_ZTestdata != 0)
	{
		//FT[Zg] = 25*131*1.046^(ZG_TEST - 1 )
		accelZ_FT_val = ( pow( 3425.65 , ( accel_ZTestdata - 1) ) ) ;

	}else
	{
		//FT[Zg] = 0

		accelZ_FT_val = 0 ;
	}

	//8) using the STR value and FT value find the change from factory trim value percentage
	//Compare the above calculated percentage with the factory given values
	//Change from factory trim of the self test response(%) = (STR -FT)/FT


	double accelZ_percent;


	accelZ_percent = ( (accelZ_STR -  accelZ_FT_val ) / accelZ_FT_val );

	if( ( accelZ_percent < 14) && (accelZ_percent > -14 ) )
		return 1 ;

	return 0;
}

/*****************************************************************
 * @fn			- MPU6050_XAccel_SelfTest_data()
 *
 * @brief		- This function is used to read the self test
 * 				  data register of X axis of Accel
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- The self test data of the X axis of Accelerometer
 *
 * @Note		- Can be used as the helper function for other self test functions
 *
 * 				- Can be used if the programmer wants to create a self test based
 * 				  function on their own
 * 				  of needed the self test register value for some other use in the
 * 				  application
 *
 *****************************************************************/
uint8_t MPU6050_XAccel_SelfTest_data()
{
	uint8_t  temp1 = 0 , temp2 = 0;
	uint8_t test = 0;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_X_REG_ADDR, &temp1);
	Read_MPU6050_I2C(mpuADDR,MPU6050_SELF_TEST_A_REG_ADDR, &temp2);

	/*
	 * self test value is of 5 bits of x axis and  is distributed in 2 register
	 * in 0x0D register we have upper 3 bits [7:5]
	 * in 0x10 register we have lower 2 bits [5:4]
	 */

	temp1 &= ~(0x1F);
	temp2 &= ~(0xCF);

	temp1 = ( temp1 >> 3);
	temp2 = (temp2 >> 4);

	test = temp1 + temp2 ;
	return test ;

}

/*****************************************************************
 * @fn			- uint8_t MPU6050_YAccel_SelfTest_data()
 *
 * @brief		- This function is used to read the self test
 * 				  data register of Y axis of Accel
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- The self test data of the Y axis of Accelerometer
 *
 * @Note		- Can be used as the helper function for other self test functions
 *
 * 				- Can be used if the programmer wants to create a self test based
 * 				  function on their own
 * 				  of needed the self test register value for some other use in the
 * 				  application
 *
 *****************************************************************/
uint8_t MPU6050_YAccel_SelfTest_data()
{
	uint8_t  temp1 = 0 , temp2 = 0;
	uint8_t test = 0;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Y_REG_ADDR, &temp1);
	Read_MPU6050_I2C(mpuADDR,MPU6050_SELF_TEST_A_REG_ADDR, &temp2);

	/*
	 * self test value is of 5 bits of y axis and  is distributed in 2 register
	 * in 0x0E register we have upper 3 bits [7:5]
	 * in 0x10 register we have lower 2 bits [3:2]
	 */

	temp1 &= ~(0x1F);
	temp2 &= ~(0xF3);

	temp1 = ( temp1 >> 3);
	temp2 = (temp2 >> 2);

	test = temp1 + temp2 ;
	return test ;

}


/*****************************************************************
 * @fn			- uint8_t MPU6050_ZAccel_SelfTest_data()
 *
 * @brief		- This function is used to read the self test
 * 				  data register of Z axis of Accel
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- The self test data of the Z axis of Accelerometer
 *
 * @Note		- Can be used as the helper function for other self test functions
 *
 * 				- Can be used if the programmer wants to create a self test based
 * 				  function on their own
 * 				  of needed the self test register value for some other use in the
 * 				  application
 *
 *****************************************************************/
uint8_t MPU6050_ZAccel_SelfTest_data()
{
	uint8_t  temp1 = 0 , temp2 = 0;
	uint8_t test = 0;

	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Z_REG_ADDR, &temp1);
	Read_MPU6050_I2C(mpuADDR,MPU6050_SELF_TEST_A_REG_ADDR, &temp2);

	/*
	 * self test value is of 5 bits of y axis and  is distributed in 2 register
	 * in 0x0E register we have upper 3 bits [7:5]
	 * in 0x10 register we have lower 2 bits [1:0]
	 */

	temp1 &= ~(0x1F);
	temp2 &= ~(0xFC);

	temp1 = ( temp1 >> 3);
	temp2 = (temp2 >> 0);

	test = temp1 + temp2 ;
	return test ;

}

/*****************************************************************
 * @fn			- uint8_t MPU6050_XGyro_SelfTest_data()
 *
 * @brief		- This function is used to read the self test
 * 				  data register of X axis of GYRO
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- The self test data of the X axis of GYROSCOPE
 *
 * @Note		- Can be used as the helper function for other self test functions
 *
 * 				- Can be used if the programmer wants to create a self test based
 * 				  function on their own
 * 				  of needed the self test register value for some other use in the
 * 				  application
 *
 *****************************************************************/
uint8_t MPU6050_XGyro_SelfTest_data()
{
	uint8_t temp = 0 ;
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_X_REG_ADDR, &temp);

	temp &= ~(0xE0  );

	return temp ;
}

/*****************************************************************
 * @fn			- uint8_t MPU6050_YGyro_SelfTest_data()
 *
 * @brief		- This function is used to read the self test
 * 				  data register of Y axis of GYRO
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- The self test data of the X axis of GYROSCOPE
 *
 * @Note		- Can be used as the helper function for other self test functions
 *
 * 				- Can be used if the programmer wants to create a self test based
 * 				  function on their own
 * 				  of needed the self test register value for some other use in the
 * 				  application
 *
 *****************************************************************/
uint8_t MPU6050_YGyro_SelfTest_data()
{
	uint8_t temp = 0 ;
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Y_REG_ADDR, &temp);

	temp &= ~(0xE0  );

	return temp ;
}
/*****************************************************************
 * @fn			- uint8_t MPU6050_ZGyro_SelfTest_data()
 *
 * @brief		- This function is used to read the self test
 * 				  data register of Z axis of GYRO
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- The self test data of the Z axis of GYROSCOPE
 *
 * @Note		- Can be used as the helper function for other self test functions
 *
 * 				- Can be used if the programmer wants to create a self test based
 * 				  function on their own
 * 				  of needed the self test register value for some other use in the
 * 				  application
 *
 *****************************************************************/
uint8_t MPU6050_ZGyro_SelfTest_data()
{
	uint8_t temp = 0 ;
	Read_MPU6050_I2C(mpuADDR, MPU6050_SELF_TEST_Z_REG_ADDR, &temp);

	temp &= ~(0xE0  );

	return temp ;
}


/*
 * functions for Sample rate of MPU
 */

/*****************************************************************
 * @fn			- MPU6050_SampleRate_DivConfig
 *
 * @brief		- This function we will config the divider for the  sample rate
 * 				  OF OUR SENSOR
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- sample rate
 *
 * @Note		- The sensor register output, FIFO output, DMP sampling
 * 				  and Motion detection are all based on the Sample Rate.
				  The Sample Rate is generated by dividing the gyroscope
				  output rate by SMPLRT_DIV:
				  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)

				  where Gyroscope Output Rate = 8kHz when the DLPF is
				  disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled.

				  The accelerometer output rate is 1kHz. This means that
				  for a Sample Rate greater than 1kHz, the same accelerometer
				  sample may be output to the FIFO, DMP, and sensor registers
				  more than once.

				  In this function we config the SMPRT register(0x19) with an 8 bit
				  unsigned value
 *
 *****************************************************************/
void MPU6050_SampleRate_DivConfig(uint8_t dividerVal)
{
	Write_MPU6050_I2C(mpuADDR, MPU6050_SMPLRT_DIV_REG_ADDR, dividerVal);
}


/*****************************************************************
 * @fn			- MPU6050_GetSampleRateVal
 *
 * @brief		- This function we get the current value of sample rate
 * 				  OF OUR SENSOR
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- sample rate
 *
 * @Note		- The sensor register output, FIFO output, DMP sampling
 * 				  and Motion detection are all based on the Sample Rate.
				  The Sample Rate is generated by dividing the gyroscope
				  output rate by SMPLRT_DIV:
				  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
				  where Gyroscope Output Rate = 8kHz when the DLPF is
				  disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled.
 *
 *****************************************************************/
uint8_t MPU6050_GetSampleRateVal()
{
	uint8_t temp1 , temp2 , SampleRate , gyro_rate;
	//reading the simple rate divider value
	Read_MPU6050_I2C(mpuADDR, MPU6050_SMPLRT_DIV_REG_ADDR, &temp1);
	//reading the gyroscope output rate
	Read_MPU6050_I2C(mpuADDR, MPU6050_CONFIG_REG_ADDR, &temp2);
	temp2 &= ( 7 << MPU6050_CFG_DLPF_COFIG_BIT);

	if(temp2 == 0 )
	{
		gyro_rate = 8;
	}else
	{
		gyro_rate = 1 ;
	}

	 SampleRate = ( gyro_rate/(1 + temp1) ) ;
	 return SampleRate;
}

//NOTE: Sample rate divider should be added or NOT in accel, gyro config function  ???
/*****************************************************************
 * @fn			- MPU6050_AccelGyroTemp_Config()
 *
 * @brief		- This function is used to accelerometer, gyroscope
 * 				  Temperature sensor
 *
 * @param[in]	- pointer to MPU6050 handle structure
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- This function receives the pointer MPU6050_Handle_struct_t type variable
 * 				  where programmer provides the values for :
 * 				  FS of gyro , FS of Accel , DPLF , MPU ADDR , SamleRate divider
 * 				  and if he wants to enable FIFO or not .
 *
 * 				- STEPS :
 * 					1) config the DLF
 * 					2) config the FS of gyro and Accelerometer
 * 					3) Enable/DISABLE the FIFO
 * 					4) Set the sample div value
 *
 *
 *****************************************************************/
void MPU6050_AccelGyroTemp_Config(MPU6050_Handle_struct_t *pConfig)
{
	uint8_t data = 0 ;
	//1. configure the DLF
	data |= (pConfig->DLPF_Val << MPU6050_CFG_DLPF_COFIG_BIT );
	Write_MPU6050_I2C(pConfig->MPU_ADDR, MPU6050_CONFIG_REG_ADDR , data );
	//2. set the FS of GYRO
	data = 0 ;
	data |= (pConfig->Gyro_FS_Val << MPU6050_ACCEL_AFS_SEL_BIT );
	Write_MPU6050_I2C(pConfig->MPU_ADDR , MPU6050_GYRO_CONFIG_REG_ADDR , data);
	//Set the FS of Accelerometer
	data = 0 ;
	data |= (pConfig->Accel_FS_Val << MPU6050_ACCEL_AFS_SEL_BIT );
	Write_MPU6050_I2C(pConfig->MPU_ADDR , MPU6050_ACCEL_CONFIG_REG_ADDR , data);
	//3. ENABLE/DISABLE FIFO of ACCELEROMETER
	data = 0;
	if( (pConfig->Accel_FIFO_EnOrDi == ENABLE ) || (pConfig->Accel_FIFO_EnOrDi == ENABLE )
			|| (pConfig->Temp_FIFO_EnOrDi == ENABLE))
	{
		data |= (pConfig->Accel_FIFO_EnOrDi << MPU6050_ACCEL_FIFO_EN_BIT );

		if(pConfig->Gyro_FIFO_EnOrDi == ENABLE)
		{
			data |= (0x7 << MPU6050_ZG_FIFO_EN_BIT );
		}
		if(pConfig->Temp_FIFO_EnOrDi == ENABLE)
			data |= (pConfig->Temp_FIFO_EnOrDi <<  MPU6050_TEMP_FIFO_EN_BIT );
		else
			data &= ~(pConfig->Temp_FIFO_EnOrDi <<  MPU6050_TEMP_FIFO_EN_BIT );

		Write_MPU6050_I2C(pConfig->MPU_ADDR , MPU6050_ACCEL_CONFIG_REG_ADDR , data);



		//enabling the fifo
		data = 0;
		data |= (1 << MPU6050_FIFO_EN_BIT );
		Write_MPU6050_I2C(pConfig->MPU_ADDR , MPU6050_USER_CTRL_REG_ADDR , data);

	}

	//4. SET the sample rate divider value
	Write_MPU6050_I2C(pConfig->MPU_ADDR, MPU6050_SMPLRT_DIV_REG_ADDR,pConfig-> SampleRateDiv);


}


/*****************************************************************
 * @fn			- void MPU6050_Accel_Config
 *
 * @brief		- This function is to configure the Accelerometer sensor
 *
 *
 * @param[in]	- pointer to the variable of MPU6050_AccelHandle_Struct_t type
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- Usually used we we only want to use accelerometer sensor of the
 * 				  our MPU60x0
 *
 * 				- in this we configure the accelerometer as per our application requirements
 *
 *
 *****************************************************************/
void MPU6050_Accel_Config(MPU6050_AccelHandle_Struct_t *Accel_config)
{
	MPU6050_DLPF_Config(Accel_config->DLF_Val);
	MPU6050_AccelFS_Config(Accel_config->FS_Val);
	MPU6050_MotDet_Config(Accel_config->Motion_thershold);
	MPU6050_FIFO_AccelEnOrDi(Accel_config->FIFO_EnOrDi);
}

/*****************************************************************
 * @fn			- void MPU6050_Gyro_Config(MPU6050_GyroHandle_Struct_t *Gyro)
 *
 * @brief		- This function is to configure the Gyroscope sensor
 *
 *
 * @param[in]	- pointer to the variable of MPU6050_GyroHandle_Struct_t type
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- Usually used we we only want to use gyroscope sensor of the
 * 				  our MPU60x0
 *
 * 				- in this we configure the gyroscope as per our application requirements
 * 					 i.e its FS value
 * 					 it's DLF value
 * 					 EnOrDi FIFO of Gyro
 *
 *
 *****************************************************************/
void MPU6050_Gyro_Config(MPU6050_GyroHandle_Struct_t *Gyro)
{

	MPU6050_DLPF_Config(Gyro->DLF_Val);
	MPU6050_GyroFS_Config(Gyro->FS_Val);

	uint8_t temp = 0;
	if(	Gyro->Gyro_FIFO_EnOrDi == ENABLE)
	{
		temp |= ( 7 << MPU6050_ZG_FIFO_EN_BIT);
		MPU6050_FIFO_EnOrDi(ENABLE);
	}else
	{
		temp &= ~( 7 << MPU6050_ZG_FIFO_EN_BIT);
		MPU6050_FIFO_EnOrDi(DISABLE);
	}

}



/*****************************************************************
 * @fn			- void MPU6050_SlaveX_Config
 *
 * @brief		- this function is used if we are connecting a slave to the
 * 				  auxiliary I2C interface of our sensor.
 * 				- function is used if a slave has to be connected, to any of the
 * 				   4 salves registers
 *
 * @param[in]	- pointer to variable of type slaveHandle_Struct
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		-  MPU6050_Slave_EnOrDi(uint8_t SlaveName , uint8_t EnOrDi );
 * 				  this function has to be called after this function to enable/disable slave
 * 				  i.e data transfer from/to slave
 *
 * 				- This function is used to configure the slave registers of the
 * 				 Required slave as only after configuring these registers our MPU60x0
 * 				 could communicate with connected Slave devices.
 *				 in this we configure the :
 *				 # SlaveName :   This is to  select from the given 4 slave options 4 slaves
 *				 				 Slave Name has to be Provided as the register address
 *				 				 depend on the slave we are working on.
 *				 # Slave address : It is the 7-bit I2C address of Slave connected
 *				 # GroupOder 	 : It specifies the grouping order of word pairs received from registers
 *				 				   It is configured according to the slave connected.
 *				 # SlvRegAddr	 : 8-bit address of the SlaveX register to/from which data transfer starts
 *				 # SlvBytSwap	 : configures byte swapping of word pairs
 *				 					NOTE : SlvBytSwap and GroupOder are inter dependent
 *				 # SlvOut_dataLen : Specifies the number of bytes transferred to and from Slave
 *				 # Slv_REG_DIS 	 : To configure the data transaction method of slave
 *				 # SlaveFIFO_EnorDi : To Enable/ Disable FIFO for that salve
 *
 *
 *@V.IMP		- I2C data transactions are performed at the Sample Rate,
 *				  The user is responsible for ensuring that I2C data transactions to and from each enabled
 *				  Slave can be completed within a single period of the Sample Rate.
 *
 *				- The I2C slave access rate can be reduced relative to the Sample Rate. This reduced
 *				  access rate is determined by I2C_MST_DLY (Register 52). Whether a slave’s access rate is
 *				  reduced relative to the Sample Rate is determined by I2C_MST_DELAY_CTRL (Register 103).
 *				  Each slave can either be accessed at the sample rate or at a reduced sample rate.
 *				  In a case where some slaves are accessed at the Sample Rate and some slaves are accessed
 *				  at the reduced rate, the sequence of accessing the slaves (Slave 0 to Slave 4) is still followed.
 *				  However, the reduced rate slaves will be skipped if their access rate dictates that they should
 *				  not be accessed during that particular cycle.
 *
 *@V.IMP		- The processing order for the slaves is fixed. The sequence followed for processing the
 *				  slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a particular Slave is disabled
 *				  it will be skipped.
 *****************************************************************/
void MPU6050_SlaveX_Config(MPU6050_SlaveHandle_struct_t *SlaveParamters)
{

	//if using write multiple data function the 0th data should be
	//the address  register we have to write
	uint8_t data[4] = {0, 0 , 0 , 0 } ;
	//NOTE: Bits order is same for Slave 0,1,2,3 just register addressare different
	data[1] = 	SlaveParamters->SlvAddr;
	data[2] =  SlaveParamters->SlvRegAddr;
	uint8_t temp = (SlaveParamters->SlvOut_dataLen & 0x0F); //as Len should be only 4 bits
	data[3] |= (temp << MPU6050_I2C_SLV1_DATALEN_BIT);
	data[3] |= (SlaveParamters->SlvBytSwap << MPU6050_I2C_SLV1_BYTE_SW_BIT );
	data[3] |= (SlaveParamters->GroupOder << MPU6050_I2C_SLV1_GRP_BIT );
	data[3] |= (SlaveParamters->Slv_REG_DIS  << MPU6050_I2C_SLV1_REG_DIS_BIT );


	if(SlaveParamters->SlaveName == MPU6050_SLAVE_0 )
	{
		data[0] = MPU6050_I2C_SLV0ADDR_REG_ADDR;
		Write_Multiple_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV0ADDR_REG_ADDR, data, 4);


	}else if (SlaveParamters->SlaveName == MPU6050_SLAVE_1)
	{
		data[0] = MPU6050_I2C_SLV1ADDR_REG_ADDR;
		Write_Multiple_MPU6050_I2C(mpuADDR,MPU6050_I2C_SLV1ADDR_REG_ADDR, data, 4);
	}else if (SlaveParamters->SlaveName == MPU6050_SLAVE_2)
	{
		data[0] = MPU6050_I2C_SLV2ADDR_REG_ADDR;
		Write_Multiple_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV2ADDR_REG_ADDR, data, 4);
	}else if (SlaveParamters->SlaveName == MPU6050_SLAVE_3)
	{
		data[0] = MPU6050_I2C_SLV3ADDR_REG_ADDR;
		Write_Multiple_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV3ADDR_REG_ADDR, data, 4);
	}else if (SlaveParamters->SlaveName == MPU6050_SLAVE_4)
	{
		//setting of slave 4 is little different
		//because its register order is little different
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV4ADDR_REG_ADDR, SlaveParamters->SlvAddr);
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV4REG_REG_ADDR, SlaveParamters->SlvRegAddr);
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV4CTRL_REG_ADDR, data[3]);
	}

	MPU6050_FIFO_SlaveEnOrDi(SlaveParamters->SlaveName, SlaveParamters->SlaveFIFO_EnorDi);
}

void MPU6050_Set_SlaveX_DataLn(uint8_t SlaveName ,uint8_t Len)
{
	uint8_t temp = 0;
	Len &= (0x0F); // as len should be a 4 bit value only

	if(SlaveName == MPU6050_SLAVE_0)
	{
		//clearing the first 4 bits that we have to write
		temp &= ~( 0xF<< MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV0CTRL_REG_ADDR, temp);

		temp |= (Len << MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV0CTRL_REG_ADDR, temp);

	}else if (SlaveName == MPU6050_SLAVE_1)
	{
		//clearing the first 4 bits that we have to write
		temp &= ~( 0xF<< MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, temp);

		temp |= (Len << MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, temp);

	}else if(SlaveName == MPU6050_SLAVE_2)
	{
		//clearing the first 4 bits that we have to write
		temp &= ~( 0xF<< MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, temp);

		temp |= (Len << MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, temp);

	}else if(SlaveName == MPU6050_SLAVE_3)
	{
		//clearing the first 4 bits that we have to write
		temp &= ~( 0xF<< MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, temp);

		temp |= (Len << MPU6050_I2C_SLV0_DATALEN_BIT );
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, temp);

	}
}
void MPU6050_Get_SlaveX_DataLn(uint8_t SlaveName ,uint8_t Len);



/*****************************************************************
 * @fn			- MPU6050_GyroFS_Config
 *
 * @brief		- This function we configure the full scale range
 * 				   of the GYRO scope
 *
 * @param[FS_val]	- full scale range we want
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we config the FS_SEL[1:0] bit of
 * 				  GYRO_CONFIG register(0x1B)
 * 				  FS_SEL selects the full scale range of
 * 				  the gyroscope outputs according to the following table.
 *
 * 				  FS_SEL		Full Scale Range
 *								(in degree/second)
 * 				  	0			+- 250
 * 				  	1			+- 500
 * 				  	2			+- 1000
 * 				  	3			+- 2000
 *
 *
 *****************************************************************/
void MPU6050_GyroFS_Config(uint8_t FS_val)
{
	uint8_t data = 0;
	data |= (FS_val << MPU6050_GYRO_FS_SEL_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_GYRO_CONFIG_REG_ADDR, data);
}

/*****************************************************************
 * @fn			- MPU6050_AccelFS_Config
 *
 * @brief		- This function we configure the full scale range
 * 				   of the accelerometer
 *
 * @param[FS_val]	- full scale range we want
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we config the FS_SEL[1:0] bit of
 * 				  ACCEL_CONFIG register(0x1C)
 * 				  FS_SEL selects the full scale range of
 * 				  the gyroscope outputs according to the following table.
 *
 * 				  FS_SEL		Full Scale Range
 *								(in g)
 * 				  	0			+- 2
 * 				  	1			+- 4
 * 				  	2			+- 8
 * 				  	3			+- 16
 *
 *
 *****************************************************************/

void MPU6050_AccelFS_Config(uint8_t FS_val)
{
	uint8_t data = 0;
	data |= (FS_val << MPU6050_ACCEL_AFS_SEL_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_CONFIG_REG_ADDR , data);
}


/*
 * for reading  Accelerometer  data
 */

/*****************************************************************
 * @fn			- MPU6050_Get_AccelData
 *
 * @brief		- This function is to read the recent most accelerometer
 * 				  measurements
 *
 *
 * @param[Xaccel]	- to store the measurement of the the X axies of accelerometer
 * @param[Yaccel]	- to store the measurement of the the Y axies of accelerometer
 * @param[Zaccel]	- to store the measurement of the the Z axies of accelerometer
 *
 * @return		- None
 *
 * @Note		- The recent most accelerometer measurements are sorted in the
 * 				  registers 0x3B to 0x40
 *
 * 				-
 *
 *****************************************************************/

void MPU6050_Get_AccelData(uint16_t *Xaccel ,uint16_t *Yaccel ,uint16_t *Zaccel)
{
	uint8_t data[6] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_XOUT_H_REG_ADDR, data, 6);
	*Xaccel = ((uint16_t)data[0] << 8);
	*Xaccel |= data[1] ;

	*Yaccel = ((uint16_t)data[2] << 8);
	*Yaccel |= data[3] ;

	*Zaccel = ((uint16_t)data[4] << 8);
	*Zaccel |= data[5] ;

}

void MPU6050_Get_X_AccelData(uint16_t *Xaccel )
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_XOUT_H_REG_ADDR, data, 2);
	*Xaccel = ((uint16_t)data[0] << 8);
	*Xaccel |= data[1] ;



}
void MPU6050_Get_Y_AccelData(uint16_t *Yaccel )
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_YOUT_H_REG_ADDR, data, 2);
	*Yaccel = ((uint16_t)data[0] << 8);
	*Yaccel |= data[1] ;
}
void MPU6050_Get_Z_AccelData(uint16_t *Zaccel)
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_ACCEL_ZOUT_H_REG_ADDR, data, 2);
	*Zaccel = ((uint16_t)data[0] << 8);
	*Zaccel |= data[1] ;
}

/*
 * For Reading Gyroscope data
 *
 */

/*****************************************************************
 * @fn			- MPU6050_Get_GyroData
 *
 * @brief		- This function is to read the recent most gyroscope
 * 				  measurements
 *
 *
 * @param[Xgyro]	- to store the measurement of the the X axis of gyroscope
 * @param[Ygyro]	- to store the measurement of the the Y axis of gyroscope
 * @param[Zgyro]	- to store the measurement of the the Z axis of gyroscope
 *
 * @return		- None
 *
 * @Note		- The recent most gyroscope  measurements are sorted in the
 * 				  registers 0x3B to 0x40
 *
 * 				-
 *
 *****************************************************************/
void MPU6050_Get_GyroData(uint16_t *Xgyro ,uint16_t *Ygyro ,uint16_t *Zgyro)
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_XOUT_H_REG_ADDR, data, 2);
	*Xgyro = ((uint16_t) data[0] << 8);
	*Xgyro = data[1];

	*Ygyro = ((uint16_t) data[2] << 8);
	*Ygyro = data[3];

	*Zgyro = ((uint16_t) data[4] << 8);
	*Zgyro = data[5];
}
void MPU6050_Get_X_GyroData(uint16_t *Xgyro )
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_XOUT_H_REG_ADDR, data, 2);
	*Xgyro = ((uint16_t) data[0] << 8);
	*Xgyro = data[1];
}
void MPU6050_Get_Y_GyroData(uint16_t *Ygyro )
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_YOUT_H_REG_ADDR, data, 2);
	*Ygyro = ((uint16_t) data[0] << 8);
	*Ygyro = data[1];
}
void MPU6050_Get_Z_GyroData(uint16_t *Zgyro)
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_GYRO_ZOUT_H_REG_ADDR, data, 2);
	*Zgyro = ((uint16_t) data[0] << 8);
	*Zgyro = data[1];
}

/*
 * for temp data
 */

/*****************************************************************
 * @fn			- MPU6050_Get_GyroData
 *
 * @brief		- This function is to read the recent most temp sensor
 * 				  measurements
 *
 *
 * @param[TempVal]	- to store the measurement of the temperature sensor
 *
 * @return		- None
 *
 * @Note		- The recent most temp  measurements are sorted in the
 * 				  registers 0x3B to 0x40
 *
 * 				-
 *
 *****************************************************************/
void MPU6050_Get_TempData(uint16_t *TempVal)
{
	uint8_t data[2] ;

	Read_Multiple_MPU6050_I2C(mpuADDR, MPU6050_TEMP_OUT_H_REG_ADDR	, data, 2);
	*TempVal = ((uint16_t) data[0] << 8);
	*TempVal = data[1];
}

/*****************************************************************
 * @fn			- MPU6050_Get_GyroData
 *
 * @brief		- This function is to read the recent most temp sensor
 * 				  measurements
 *
 *
 * @param[TempVal]	- to store the measurement of the temperature sensor
 *
 * @return		- None
 *
 * @Note		- The recent most temp  measurements are sorted in the
 * 				  registers 0x3B to 0x40
 *
 * 				- If a slave is disabled at any time, the space initially
 * 				  allocated to the slave in the EXT_SENS_DATA register, will
 * 				  remain associated with that slave. This is to avoid dynamic
 * 				  adjustment of the register allocation.
 * 				  Register Allocation for Dynamic Disable vs. Normal Disable
				  The allocation of the EXT_SENS_DATA registers is recomputed
				  only when (1) all slaves are disabled, or
				   	   	     (2) the I2C_MST_RST bit is set
 *
 *****************************************************************/
void MPU6050_Get_SlavexData(uint16_t SlaveName );

/*	uint8_t data;
	if (SlaveName == MPU6050_SLAVE_1)
	{

	}else if (SlaveName == MPU6050_SLAVE_2)
	{

	}else if(SlaveName == MPU6050_SLAVE_3)
	{

	}else if(SlaveName == MPU6050_SLAVE_4 )
	{
		Read_MPU6050_I2C(mpuADDR, RegADDR, &data);
	}*/


/*****************************************************************
 * @fn			- MPU6050_Get_GyroData
 *
 * @brief		- This function is to read the recent most temp sensor
 * 				  measurements
 *
 *
 * @param[TempVal]	- to store the measurement of the temperature sensor
 *
 * @return		- None
 *
 * @Note		- The recent most temp  measurements are sorted in the
 * 				  registers 0x3B to 0x40
 *
 * 				-
 *
 *****************************************************************/
void MPU6050_Write_SlavexData(uint8_t data ,uint8_t SlaveName )
{
	if(SlaveName == MPU6050_SLAVE_0 )
	{
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV0_DO_REG_ADDR, data);

	}else if( SlaveName == MPU6050_SLAVE_1 )
	{
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1_DO_REG_ADDR ,  data);
	}else if(SlaveName == MPU6050_SLAVE_2 )
	{
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV2_DO_REG_ADDR , data);
	}else if(SlaveName == MPU6050_SLAVE_3 )
	{
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV3_DO_REG_ADDR , data);
	}else if(SlaveName == MPU6050_SLAVE_4 )
	{
		Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV4DO_REG_ADDR	, data);
	}
}



/*
 * other helper functions
 */
/*****************************************************************
 * @fn			- MPU6050_ClkSource_Select
 *
 * @brief		- This function is used to select the clk source of
 * 				  the MPU60x0
 *
 * @param[ClkName]	- Name of the CLK source
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we configure the CLKSEL[2:0] bit of the
 * 				  PWR_MGMT_1 register(0x6B)
 *
 * 				  Upon power up, the MPU-60X0 clock source defaults
 * 				  to the internal oscillator.
 * 				  It is recommended that the device be configured to use one of the
 * 				  gyroscopes(ext clk) as the clk referance for improved stability.
 * 				  for CLKSEL values refer table on page 41 of register manual
 *
 *
 * @Note		- The MPU-60X0 has a flexible clocking scheme,
 *				  allowing a variety of internal or external clock sources
 *				  to be used for the internal synchronous circuitry.
 *				  This synchronous circuitry includes the signal conditioning
 *				  and ADCs, the DMP, and various control circuits and registers.
 *				  An on-chip PLL provides flexibility in the allowable inputs
 *				  for generating this clock.
 * 				  Allowable internal sources for generating the internal clock are:
 *					- An internal relaxation oscillator
 * 					- Any of the X, Y, or Z gyros
 *				   Allowable external clocking sources are:
 * 					- 32.768kHz square wave
 * 					- 19.2MHz square wave
				  Selection of the source for generating the internal synchronous
				  clock depends on the availability of external sources and the
				  requirements for power consumption and clock accuracy.
 *
 *****************************************************************/
void MPU6050_ClkSource_Select(uint8_t ClkName)
{
	uint8_t data = 0;

	data |=(ClkName << MPU6050_CKL_SELECT_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR, data);

}



/*
 * Functions related to the FIFO
 * Data stored inside the sensor data registers (Registers 59 to 96)
 * will be loaded into the FIFO buffer if a sensor’s respective
 * FIFO_EN bit is set to 1 in this register.
 *
 * When a sensor’s FIFO_EN bit is enabled in this register, data from
 * the sensor data registers will be loaded into the FIFO buffer.
 * The sensors are sampled at the Sample Rate as defined
 */



/*****************************************************************
 * @fn			- MPU6050_Slave_DataTrans_EnOrDi
 *
 * @brief		- This function enables or disables the slve of the MPU60x0
 * 			   	  from data transfer
 *
 * @param[SlaveName]	- Name of the slave to be configured
 * @param[EnOrDi]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- In this function we configure the I2C_SLV0_EN bit
 * 				  of the I2C_SLVx_CTRL register
 *
 * 				  I2C_SLVx_EN Bit :
 * 				  - When set to 1, this bit enables Slave x for
 * 				    data transfer operations.
 * 				  - When cleared to 0, this bit disables Slave x from
 * 				    data transfer operations.
 *
 *****************************************************************/
void MPU6050_Slave_EnOrDi(uint8_t SlaveName , uint8_t EnOrDi )
{
	if (SlaveName == MPU6050_SLAVE_0 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_I2C_SLV0_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV0CTRL_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_I2C_SLV0_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV0CTRL_REG_ADDR, data);
		}

	}else if (SlaveName == MPU6050_SLAVE_1 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_I2C_SLV1_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_I2C_SLV1_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV1CTRL_REG_ADDR, data);
		}


	}else if(SlaveName == MPU6050_SLAVE_2 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_I2C_SLV2_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV2CTRL_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_I2C_SLV2_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV2CTRL_REG_ADDR, data);
		}


	}else if (SlaveName == MPU6050_SLAVE_3 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_I2C_SLV3_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV3CTRL_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_I2C_SLV3_EN_BIT);
			Write_MPU6050_I2C(mpuADDR,MPU6050_I2C_SLV3CTRL_REG_ADDR, data);
		}


	}else if (SlaveName == MPU6050_SLAVE_4 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_I2C_SLV4_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_SLV4CTRL_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_I2C_SLV4_EN_BIT);
			Write_MPU6050_I2C(mpuADDR,MPU6050_I2C_SLV4CTRL_REG_ADDR, data);
		}


	}


}

/*****************************************************************
 * @fn			- MPU6050_FIFO_GyroEnOrDi
 *
 * @brief		- This function enables or disables FIFO
 * 			  	  for the given gyroscope axis
 *
 * @param[GyroAxis]	- name of the GYRo axis whose fifo to be configured
 * @param[EnOrDi]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- In this we configure the XG_ FIFO_EN or YG_ FIFO_EN
 * 				  or ZG_ FIFO_EN of FIFO_EN register(0x23)
 * 				  When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L
 *				  to be written into the FIFO buffer.
 *				  then configured to 0 the data of data register will
 *				  not be written into the FIFO buffer
 *****************************************************************/
void MPU6050_FIFO_GyroEnOrDi(uint8_t GyroAxis , uint8_t EnOrDi )
{
	uint8_t data = 0 ;

	if(GyroAxis ==  MPU6050_GYRO_X_AXIS)
	{
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_XG_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR , data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 << MPU6050_XG_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR , data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}
	}else if( MPU6050_GYRO_Y_AXIS)
	{
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_YG_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR , data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 << MPU6050_YG_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR , data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}


	}else if( MPU6050_GYRO_Z_AXIS)
	{
		if(EnOrDi == ENABLE)
		{
			data |=(1 << MPU6050_ZG_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR , data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 << MPU6050_ZG_FIFO_EN_BIT );
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR , data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}

	}
}

/*****************************************************************
 * @fn			- MPU6050_FIFO_AccelEnOrDi
 *
 * @brief		- This function enables or disables FIFO
 * 			  	  for the accelerometer
 *
 * @param[in]	- N/A
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- in this we configure the ACCEL_ FIFO_EN bit
 *    			  of FIFO_EN register(0x23)
 *    			  When set to 1, this bit enables ACCEL_XOUT_H,
 *    			  ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
 *    			  ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64)
 *    			  to be written into the FIFO buffer
 *
 *****************************************************************/
void MPU6050_FIFO_AccelEnOrDi( uint8_t EnOrDi )
{
	uint8_t data = 0 , temp = 0;
	if(EnOrDi == ENABLE)
	{
		data |= (1 << MPU6050_ACCEL_FIFO_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);
		temp |= (1 << MPU6050_FIFO_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR , temp);
	}else
	{
		data &= ~(1 << MPU6050_ACCEL_FIFO_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);

		temp &= ~(1 << MPU6050_FIFO_EN_BIT	 );
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR , temp);
	}
}


/*****************************************************************
 * @fn			- MPU6050_FIFO_SlaveEnOrDi
 *
 * @brief		- This function enables or disables FIFO
 * 			  	  for the slave
 *
 * @param[SlaveName]	- N/A
 * @param[EnOrDi]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- in this we configure the ACCEL_ FIFO_EN bit
 *    			  of FIFO_EN register(0x23)
 *    			  When set to 1, this bit enables ACCEL_XOUT_H,
 *    			  ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
 *    			  ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64)
 *    			  to be written into the FIFO buffer
 *
 *****************************************************************/
void MPU6050_FIFO_SlaveEnOrDi(uint8_t SlaveName , uint8_t EnOrDi )
{
	if (SlaveName == MPU6050_SLAVE_0 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_SLV0_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 << MPU6050_SLV0_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}

	}else if (SlaveName == MPU6050_SLAVE_1 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 <<  MPU6050_SLV1_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 <<  MPU6050_SLV1_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}


	}else if(SlaveName == MPU6050_SLAVE_2 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_SLV2_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR,MPU6050_FIFO_EN_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 << MPU6050_SLV2_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_FIFO_EN_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}


	}else if (SlaveName == MPU6050_SLAVE_3 )
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_SLV3_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_MST_CTRL_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(ENABLE);
		}else
		{
			data &= ~(1 << MPU6050_SLV3_FIFO_EN_BIT);
			Write_MPU6050_I2C(mpuADDR,MPU6050_I2C_MST_CTRL_REG_ADDR, data);
			MPU6050_FIFO_EnOrDi(DISABLE);
		}


	}


}


/*****************************************************************
 * @fn			- MPU6050_MotDet_Config
 *
 * @brief		- This function we configures the Motion Detection Threshold
 * 			  	  value
 *
 * @param[thresVal]	- thershold value
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- in this we configure the MOT_THR register(1F) with
 * 				  8-bit unsigned value which  Specifies the Motion detection
 * 				  threshold.
 *
 * 				  This register configures the detection threshold for Motion
 * 				  interrupt generation.Motion is detected when the absolute value
 * 				  of any of the accelerometer measurements exceeds this Motion
 * 				  detection threshold.The Motion interrupt will indicate the axis
 * 				  and polarity of detected motion in MOT_DETECT _STATUS
 *
 *****************************************************************/
void MPU6050_MotDet_Config(uint8_t thresVal)
{
	Write_MPU6050_I2C(mpuADDR, MPU6050_MOT_THR_REG_ADDR, thresVal);
}

// make a function to read thershold value
//make function for free fall detect duration
/*
 * communication I2C
 */

/*****************************************************************
 * @fn			- MPU6050_I2C_BYPASS
 *
 * @brief		- This function is to bypass the auxiliary bus so that
 * 				  System processor can directly access the Slave.
 *
 * @param[in]	- N/A
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- here we config I2C _BYPASS _EN bit of
 * 				  INT_PIN_CFG (0x37)
 * 				  When this bit is equal to 1 and I2C_MST_EN is equal to 0,
 * 				  the host application processor will be able to directly
 * 				  access the auxiliary I2C bus of the MPU-60X0.
 *
 *				  When this bit is equal to 0, the host application processor
 *				  will not be able to directly access the auxiliary I2C bus
 *				  of the MPU-60X0 regardless of the state of I2C_MST_EN
 *
 *
 *****************************************************************/
void MPU6050_I2C_BYPASS(uint8_t EnOrDi)
{
	uint8_t data = 0;
	if( EnOrDi == ENABLE)
	{
		data |= (ENABLE << MPU6050_I2C_BYPASS_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR, data);

	}else
	{
		data &=~(1 << MPU6050_I2C_BYPASS_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR , data);
	}
}


/*****************************************************************
 * @fn			- MPU6050_I2C_MasterCtrl_config
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
void MPU6050_I2C_MasterCtrl_config();



/*****************************************************************
 * @fn			- GMPU6050_I2C_MasterClk_Val
 *
 * @brief		- This function configure the CLK(SCL) which
 * 				  MPU60x0 produces when it act as a master for
 * 				  slaves connected to its auxailary I2C bus
 *
 * @param[MasterClkSp]	- Base address of the GPIO peripheral
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- in this we configure the I2C_MST_CLK bit of
 * 				  I2C_MST_CTRL resister(0x24) @MasterClkSp
 *
 *****************************************************************/
void MPU6050_I2C_MasterClk_Val(uint8_t MasterClkSp)
{
	uint8_t data = 0 ;
	data |= (MasterClkSp << MPU6050_I2C_MST_CLK_BIT	);
	Write_MPU6050_I2C(mpuADDR, MPU6050_I2C_MST_STATUS_REG_ADDR , data);
}

/*****************************************************************
 * @fn			- MPU6050_I2C_MultiMaster_EnOrDi
 *
 * @brief		- This function enables or disables master mode
 * 				   of MPU60x0 's auxa;iary bus

 *
 * @param[in]	-N/A
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- In this function we config the MPU6050_I2C_MSTER_EN_BIT
 * 				  bit of the USER_CTRL register.
 * 				  When set to 1, this bit enables I2C Master Mode.
 * 				  When this bit is cleared to 0, the auxiliary I2C bus
 * 				  lines (AUX_DA and AUX_CL) are logically driven by the
 * 				  primary I2C bus (SDA and SCL).
 *
 *
 *****************************************************************/
void MPU6050_I2C_Multi_EnOrDi(uint8_t EnOrDi)
{
	uint8_t data = 0;
	if( EnOrDi == ENABLE)
	{
		data |= (ENABLE << MPU6050_I2C_MSTER_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR, data);

	}else
	{
		data &=~(1 << MPU6050_I2C_MSTER_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR, data);
	}
}

/*
 * for configuring the interrupts
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
void MPU6050_IntPin_Config( MPU6050_IntPinHandle_t *pHandle)
{
	uint8_t temp = 0 ;
	temp |= (pHandle->PinClearType << MPU6050_INT_PIN_RD_CLEAR_BIT);
	temp |= (pHandle->PinLevel << MPU6050_INT_PIN_LEVEL_BIT);
	temp |= (pHandle->PinType << MPU6050_INT_PIN_TYPE_BIT );
	temp |= (pHandle->OutState << MPU6050_INT_PIN_LATCH_EN_BIT);

	Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR, temp);

}




/*****************************************************************
 * @fn			- MPU6050_IntPin_LevelConfig
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given GPIO port
 *
 * @param[in]	- N/A
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void MPU6050_IntPin_LevelConfig(uint8_t highOrLow)
{
	uint8_t data = 0;
	if(highOrLow == HIGH)
	{
		data &= ~(1 << MPU6050_INT_PIN_LEVEL_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR, data);
	}else
	{
		data |= (1 << MPU6050_INT_PIN_LEVEL_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR, data);

	}
}

/*****************************************************************
 * @fn			- MPU6050_IntPin_ModeConfig
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void MPU6050_IntPin_ModeConfig(uint8_t Mode)
{
	uint8_t data = 0;
	if(Mode == MPU6050_INT_PIN_MODE_OD)
	{
		data |= (1 << MPU6050_INT_PIN_TYPE_BIT);
		Write_MPU6050_I2C(mpuADDR,  MPU6050_INT_PIN_CFG_REG_ADDR, data);
	}else
	{
		data &= ~(1 << MPU6050_INT_PIN_TYPE_BIT);
		Write_MPU6050_I2C(mpuADDR,  MPU6050_INT_PIN_CFG_REG_ADDR, data);

	}
}

/*****************************************************************
 * @fn			- MPU6050_IntPin_LatchEnOrDi
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given GPIO port
 *
 * @param[in]	- N/A
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void MPU6050_IntPin_LatchEnOrDi(uint8_t EnOrDi)
{
	uint8_t data = 0 ;
	if(EnOrDi == ENABLE)
	{
		data |= (1 <<  MPU6050_INT_PIN_LATCH_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR, data);
	}else
	{
		data |= (1 <<  MPU6050_INT_PIN_LATCH_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR, data);


	}
}

/*****************************************************************
 * @fn			- MPU6050_FSYNC_INTLevel
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given GPIO port
 *
 * @param[in]	- N/A
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void MPU6050_FSYNC_INTLevel(uint8_t highOrLow)
{
	uint8_t data = 0;
	if(highOrLow == ENABLE)
	{
		data |= (1 <<  MPU6050_INT_PIN_LATCH_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_FSYNC_INT_LEVEL_BIT, data);
	}else
	{
		data |= (1 <<  MPU6050_INT_PIN_LATCH_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_FSYNC_INT_LEVEL_BIT, data);


	}
}

/*****************************************************************
 * @fn			- MPU6050_GetIT_Status
 *
 * @brief		- This function enables or disables peripheral
 * 			  	  clock for the given GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
uint8_t MPU6050_GetIT_Status(uint8_t InteruptName)
{
	uint8_t data = 0;
//	uint8_t val = 0;
	if(InteruptName ==  MPU6050_DATA_READY_IT)
	{
		Read_MPU6050_I2C(mpuADDR, MPU6050_INT_STATUS_REG_ADDR, &data);

	}else if(InteruptName ==  MPU6050_I2C_MASTER_IT)
	{
		Read_MPU6050_I2C(mpuADDR, MPU6050_INT_STATUS_REG_ADDR, &data);

	}else if(InteruptName ==  MPU6050_FIFO_OFLOW_IT)
	{
		Read_MPU6050_I2C(mpuADDR, MPU6050_INT_STATUS_REG_ADDR, &data);

	}else if(InteruptName ==  MPU6050_MOTION_DETECTION_IT)
	{
		 Read_MPU6050_I2C(mpuADDR, MPU6050_INT_STATUS_REG_ADDR, &data);

	}
	return data;
}



/*****************************************************************
 * @fn			- void MPU6050_ITx_EnOrDi(uint8_t InteruptName , uint8_t EnOrDi);
 *
 * @brief		- This function is to Enable/Disable the interrupt
 * 			       of the  MPU .
 *
 * @param[InteruptName]	- name of the Interrupt that we want to config
 * @param[EnOrDi]		- To Enable or Disable the bit
 *
 * @return		- None
 *
 * @Note		- MPU60x0 has a programmable interrupt system which can
 * 				  generate an interrupt signal on the INT pin and the
 * 				  interrupt status flags indicate the source of an
 * 				  interrupt.
 *
 * 				  Interrupt sources :
 *
 * 				  Interrupt Name 	|		Module
 * 				  -----------------------------------
 * 				  FIFO Overflow		|		FIFO
 * 				  Data Ready		|		Sensor register
 * 				  Motion detection  |
 * 				  I2C master 		|		I2C master
 *
 * 				  Interrupt sources required can be ENABLED/DISABLED
 * 				  from the INT_ENABLE Register (0x38)
 * 				  and to read the status of the interrupt
 * 				  we refer to the INT_STATUS Register (0x3A)
 *
 * @Note		- BEFORE ENABLING THE  INTEERUPT WE SHOULD CONFIG
 * 				  INT PIN . i.e we have to config the level , mode ,
 * 				  latch et. of the INT pin we want when a interrupt ouccers
 *
 * @Note		- Program flow for interrupt :
 * 				  first send the signal to config the INT pin of the MPU
 * 				  Then send signals to Enable the required interrupt
 *
 * 				  we connect the INT pin of the MPU60x0 to our MCU
 * 				  say the setting of  INT pin is active low when
 * 				  interrupt occurs in the MPU , then config the
 * 				  GPIO of MPU to produce interrupt when
 * 				  pin is pulled low .
 * 				  Then in the GPIO handler we send signal to MPU60x0 to
 * 				  reads its  INT_STATUS Register then check which bit
 * 				  of the register is set , then send the signal for the
 * 				  required operation
 *
 *
******************************************************************/
void MPU6050_ITx_EnOrDi(uint8_t InteruptName , uint8_t EnOrDi)
{
	if(InteruptName ==  MPU6050_DATA_READY_IT)
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_DATA_RDY_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_DATA_RDY_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);

		}

	}else if(InteruptName ==  MPU6050_I2C_MASTER_IT)
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_I2C_MST_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_I2C_MST_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);

		}

	}else if(InteruptName ==  MPU6050_FIFO_OFLOW_IT)
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_FIFO_OFLOW_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);
		}else
		{
			data &= ~(1 << MPU6050_FIFO_OFLOW_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);

		}

	}else if(InteruptName ==  MPU6050_MOTION_DETECTION_IT)
	{
		uint8_t data = 0;
		if(EnOrDi == ENABLE)
		{
			data |= (1 << MPU6050_MOTION_DET_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);
		}else
		{
			data &= ~(1 <<MPU6050_MOTION_DET_IT_EN_BIT);
			Write_MPU6050_I2C(mpuADDR, MPU6050_INT_ENABLE_REG_ADDR, data);

		}

	}
}


/*
 * other reset related functions
 */


/*****************************************************************
 * @fn			- MPU6050_AccelSenor_Reset()
 *
 * @brief		- This function resets the signal paths of ACCEL sensor
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we configure the ACCEL_RESET (bit 1 ) bit of the
 * 				  SIGNAL_PATH_RESET register ( 0x68)
 * 				  The reset will revert the signal path analog to digital
 * 				  converters and filters to their power up configurations.
 * 				- This register does not clear the sensor registers
*
 *****************************************************************/
void MPU6050_AccelSenor_Reset()
{
	uint8_t data = 0 ;
	data |= (1 << MPU6050_ACCEL_SIGNAL_RESET_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_SIGNAL_RESET_REG_ADDR,data );
}

/*****************************************************************
 * @fn			- MPU6050_GyroSenor_Reset()
 *
 * @brief		- This function resets the signal paths of GYRO sensor
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we configure the GYRO_RESET (bit 2) bit of the
 * 				  SIGNAL_PATH_RESET register ( 0x68)
 * 				  The reset will revert the signal path analog to digital
 * 				  converters and filters to their power up configurations.
 * 				- This register does not clear the sensor registers
 * *
 *****************************************************************/
void MPU6050_GyroSenor_Reset()
{
	uint8_t data = 0 ;
	data |= (1 << MPU6050_GYRO_SIGNAL_RESET_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_SIGNAL_RESET_REG_ADDR,data );
}

/*****************************************************************
 * @fn			- MPU6050_TempSenor_Reset()
 *
 * @brief		- This function resets the signal paths of temp sensor
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- In this we configure the TEMP_RESET (bit 0) bit of the
 * 				  SIGNAL_PATH_RESET register ( 0x68)
 * 				  The reset will revert the signal path analog to digital
 * 				  converters and filters to their power up configurations.
 * 				- This register does not clear the sensor registers
 *
 *****************************************************************/
void MPU6050_TempSenor_Reset()
{
	uint8_t data = 0 ;
	data |= (1 << MPU6050_TEMP_SIGNAL_RESET_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_SIGNAL_RESET_REG_ADDR,data );
}

/*****************************************************************
 * @fn			-  MPU6050_FIFO_Reset()
 *
 * @brief		- This function reset the FIFO buffer the MPU60x0
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		- This function is to  reset the FIFO buffer
 * 				in this FIFO_RESET bit of the USER_CTRL(6A) is configured
 * 				This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0.
 * 				This bit automatically clears to 0 after the reset has been triggered
 *
 *****************************************************************/
void MPU6050_FIFO_Reset()
{
	uint8_t data = 0 ;
	//first we have disable the fifo buffer
	data = 0x04 ;//fifo enable bit is 6 and fifo reset bit is 2
	data |= (1 << MPU6050_FIFO_RESET_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR,data );
}

/*****************************************************************
 * @fn			- MPU6050_I2C_MST_Reset
 *
 * @brief		- This function resets the  master functionality  of the
 * 				auxiliary I2C  of our MPU60x0
 *
 *
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- None
 *
 * @Note		-  Here , MST(master) is the master functionality of the
 * 					auxiliary I2C of the MPU60x0 sensor , where MPU60x0
 * 					act as the master
 *
 * 				- when we want to reset the master we use this function
 * 				  in this we set the I2C_MST_RESET bit  of the USER_CTRL(6A)
 * 				  register
 * 				  I2C_MST_RESET bit :
 * 				  	- when set to 1 This bit resets the I2C Master
 * 				  	  while I2C_MST_EN equals 0
 * 				  This bit automatically clears to 0 after the reset
 * 				  has been triggered.
 *
 *
 *****************************************************************/
void MPU6050_I2C_MST_Reset()
{
	uint8_t data = 0 ;
	//first we have disable the i2c_mst if we want to rest it
	data |= 0x02 ; //master enable bit is 5 and master reset bit is 1
	//data |= (1 << MPU6050_I2C_MSTER_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR,data );
}

/*****************************************************************
 * @fn			- MPU6050_SIG_COND_Reset
 *
 * @brief		- This function is to reset the signal condition
 * 				  Path in our MPU60x0 using this bit for reseting
 * 				  registers of the sensor also get reseted.
 *
 * @param[in]	- N/A
 * @param[in]	- N/A
 *
 * @return		- N/A
 *
 * @Note		- In this we configure the SIG_COND_RESET bit of the
 * 				  USER_CTRL register
 *
 * 				  When set to 1 : it resets the signal paths for all
 * 				  sensors ( gyro , accelero , temp ) it also clears
 * 				  the sensor registers.
 * 				  This bit is automatically clears to 0 after rest
 * 				  has been triggered.
 *
 * 				  IF WE WANT TO RESET ONLY THE SIGNAL PATH AND NOT THE
 * 				  SENSOR REGISTER WE USE THE SIGNAL_PATH_RESET REGISTER
 * 				  BITS TO RESET THE SIGNAL OF GYRO , ACCEL , TEMP SENSOR
 * 				  i.e use functions  MPU6050_TempSenor_Reset()
 * 				  					 MPU6050_GyroSenor_Reset()
 * 				  					 MPU6050_AccelSenor_Reset()
 * 				 To reset only the signal path not the registers of
 * 				 these sensors.
 *
 *
 *****************************************************************/
void MPU6050_SIG_COND_Reset()
{
	uint8_t data = 0 ;
	data |= (1 << MPU6050_SIG_COND_RESET_BIT);
	Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR,data );
}


/*****************************************************************
 * @fn			- MPU6050_FIFO_EnOrDi
 *
 * @brief		- This function is to enable and disable the FIFO
 * 				  buffer of our sensor
 *
 * @param[EnOrDi]	- ENABLE/DISABLE
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- FIFO act as the buffer for the sensor
 * 				  data from the sensor register and the slaves in made
 * 				  to store in the FIFO if it is enabled it helps I2C to burst Read the
 * 				  data rather then indidually read selected data from the sensore register
 *
 * 				  FIFO act as buffer for the gyro,accel,temp sensor as well as the Slaves
 * 				  connected to the  auxiliary i2c of the MPU60x0 this buffer help the
 * 				  application processor to perform the buffer read operation so it save the
 * 				  application processor. Also the FIFO buffer helps DMA to act on the data
 *
 *
 * 				  FIFO_EN(bit 6) bit of USER_CTRL(6A) register help us to
 * 				  enable and disable the FIFO buffer
 * 				  FIFO_EN - 0 : The FIFO buffer is disabled . Now FIFO buffer
 * 				  				cannot be read / write
 * 				  FIFO_EN - 1 : This bit enables FIFO operations
 *
 *
 *****************************************************************/
void MPU6050_FIFO_EnOrDi(uint8_t EnOrDi)
{
	uint8_t data = 0;
	if(EnOrDi == ENABLE)
	{
		data |= (1 << MPU6050_FIFO_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR, data);
	}else
	{
		data &= ~(1 << MPU6050_FIFO_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR, data);

	}

}

/*****************************************************************
 * @fn			- MPU6050_I2C_MaterMode_EnOrDi
 *
 * @brief		- This function is to configure the master mode of MPU60X0
 * 					sensore for its auxaliary bus
 *
 * @param[EnOrDi]	- enable/disable the mater mode of auxaliary I2C bus of sensor
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- In this we configure the master mode of auxiliary bus
 * 				  of the MPU using I2C_MST_EN bit of USER_CTRL register (6A)
 *
 * 				- When I2C_MST_EN is set to 1, I2C Master Mode is enabled.
 * 				  In this mode, the MPU-60X0 acts as the I2C Master to
 * 				  the external sensor slave devices on the auxiliary I2C bus.
 * 				  When I2C_MST_EN bit is cleared to 0, the auxiliary I2C bus
 * 				  lines (AUX_DA and AUX_CL) are logically driven by the primary
 * 				  I2C bus (SDA and SCL).
 *
 * @Note		- For host processor to directly access the auxiliary I2C of the sensor
 * 				  we have to configure the I2C_BYPASS_EN bit of INT_PIN_CFG register(0x37)
 * 				  I2C_BYPASS_EN bit :
 *
 * 				  - When this bit is equal to 1 and I2C_MST_EN (Register 0x6A bit[5])
 * 				    is equal to 0, the host application processor will be able to directly
 * 				    access the auxiliary I2C bus of the MPU-60X0.
 *
 * 				  - When this bit is equal to 0, the host application processor
 * 				    will not be able to directly access the auxiliary I2C bus of the
 * 				    MPU-60X0 regardless of the state of I2C_MST_EN (Register 0x6A bit[5]).
 *
 * @Note		- why it is made like this i.e application processor to act as
 * 					master we need to config 2 bits , 1 I2C_MST_EN to disable
 * 					MPU60x0 from acting as master and other I2C_BYPASS_EN to enable
 * 					application processor to act as master
 * 					beacues their could be a instance where we want any other master for the
 * 					slave sensor rather than our application processor.
 *
 *****************************************************************/
void MPU6050_I2C_MaterMode_EnOrDi(uint8_t EnOrDi)
{
	uint8_t data1 = 0 ;
	uint8_t data2 = 0 ;
	if(EnOrDi == ENABLE)
	{
		data1 &= ~(1 << MPU6050_I2C_BYPASS_EN_BIT );
		data2 |= (1 << MPU6050_I2C_MULT_MAST_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR, data1);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR , data2);
	}else
	{
		data1 |= (1 << MPU6050_I2C_BYPASS_EN_BIT );
		data2 &= ~(1 << MPU6050_I2C_MULT_MAST_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_USER_CTRL_REG_ADDR, data1);
		Write_MPU6050_I2C(mpuADDR, MPU6050_INT_PIN_CFG_REG_ADDR , data2);

	}

}

/*****************************************************************
 * @fn			- MPU6050_CycleMode_config
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
void MPU6050_CycleMode_config(uint8_t temp)
{
	//TODO
}




/*****************************************************************
 * @fn			- MPU6050_SleepMode_EnOrDi
 *
 * @brief		- This function enables or disables sleep mode
 * 			  	  of our MPU6050
 *
 * @param[EnOrDi]	- Base address of the GPIO peripheral
 * @param[in]	-
 *
 * @return		- None
 *
 * @Note		- SLEEP mode is to save power i.e decrease power consumption
 * 				- SLEEP mode is configured by setting sleep bit of
 * 				  Power Management 1 register [PWR_MGMT_1] (0x6B)
 * 				  SLEEP bit - 1 : sleep mode is enabled
 * 				  SLEEP bit - 0 : sleep mode is disable
 * 				  when sleep mode is disabled the device i.e MPU6050 consumes
 * 				  less power as now it will not detect the accel or gyro sensor
 * 				  changes so we can't read their new changed output
 *
 *
 *****************************************************************/
void MPU6050_SleepMode_EnOrDi(uint8_t EnOrDi)
{
	uint8_t data = 0;
	if(EnOrDi == ENABLE)
	{
		data |= (1 << MPU6050_SLEEP_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR, data);
	}else
	{
		data &= ~(1 << MPU6050_SLEEP_EN_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR, data);

	}
}



/*****************************************************************
 * @fn			- MPU6050_TempSenor_EnOrDi()
 *
 * @brief		- This function is to enable/disable the temp
 * 				  sensor of our MPU
 *
 * @param[uint8_t EnOrDi]	- ENABLE/DISABLE
 * @param[in]	-
 *
 * @return		- None
 *
 *
 * @Note		- Our MPU6050 have a in built temp sensor to measure the
 * 				  temp of the MPU die as the temp variation can effect the measurement
 * 				  of the gyro and accel values so in presion apllication the
 * 				  it is very important to know the current temp of the sensor die
 *
 * 				-Sometimes we want to disable  temp sensor like to save power
 * 				 or the enviroment in which device is operating the temp condtions good
 * 				 So, disable/enable the sensor we have to configure the
 * 				 TEMP_DIS bit(3) of the PWR_MGMT_1()0x6B register
 * 				 when TEMP_DIS bit : 1 disables the temperature sensor
 * 				 when TEMP_DIS bit : 0 enable the temperature sensor
 *
 *
 *****************************************************************/
void MPU6050_TempSenor_EnOrDi(uint8_t EnOrDi )
{
	uint8_t data = 0;
	if(EnOrDi == ENABLE)
	{
		data &= ~(1 << MPU6050_TEMP_SENSOR_DI_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR, data);
	}else
	{
		data |= (1 << MPU6050_TEMP_SENSOR_DI_BIT);
		Write_MPU6050_I2C(mpuADDR, MPU6050_PWR_MGMT_1_REG_ADDR, data);

	}
}










/*========================================================================================== */



void Read_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t *data )
{
	uint8_t buffer[1] = { RegADDR } ;
	I2C_MasterSendData(System_I2C_ForMPU, buffer, 1, mpuADDR, ENABLE);

	I2C_MasterReceiveData(System_I2C_ForMPU, data, 1, mpuADDR, DISABLE);


}
void Write_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t data)
{
		uint8_t buffer[2] = { RegADDR , data };
		I2C_MasterSendData(System_I2C_ForMPU, buffer, 2, mpuADDR, DISABLE);

}
void Write_Multiple_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t *data , uint8_t len)
{
		//in this the RegAddr is included in the data recervied
	//done for easy configuring
		I2C_MasterSendData(System_I2C_ForMPU, data,  len , mpuADDR, DISABLE);

}

void Read_Multiple_MPU6050_I2C(uint8_t mpuADDR , uint8_t RegADDR , uint8_t *data , uint8_t Len )
{
	I2C_MasterSendData(System_I2C_ForMPU, &RegADDR, 1, mpuADDR, ENABLE);
//	for(uint32_t  i = 0 ; i < 3000 ; i ++ );
	I2C_MasterReceiveData(System_I2C_ForMPU, data, Len,mpuADDR, DISABLE);
//	System_I2C_ForMPU->pI2Cx->CR1 &= ~(1 << I2C_CR1_STOP);
}



