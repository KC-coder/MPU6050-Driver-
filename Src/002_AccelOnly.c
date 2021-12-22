/**
 ******************************************************************************
 * @file           : main.c
 * @author         : KAMAL CHOPRA
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
*/

#include "MPU6050_Driver.h"
#include "stm32F407xx_I2C_driver.h"
#include "stm32F407xx_gpio_driver.h"
#include "lcd.h"
int main(void)
{

/*	System_I2C_ForMPU->pI2Cx = I2C1;
	System_I2C_ForMPU->I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	System_I2C_ForMPU->I2C_Config.I2C_DeviceAddress = MY_ADDR;
	System_I2C_ForMPU->I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	System_I2C_ForMPU->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM4K;
	System_I2C_ForMPU->I2C_Config.slaveADDRmode = I2C_SLAVE_ADDR_MODE_7BITS;

	I2C_PeriClockControl(I2C1, ENABLE);
	I2C_Init(&I2C1Handle);

	System_I2C_ForMPU->DevAddr = I2C1 ;
	System_I2C_ForMPU->I2C_Config.I2C_AckControl
	System_I2C_ForMPU->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM4K;*/

    /* Loop forever */

	//1.setup the I2C to be used

	//2. setup the LCD to be used

	//3. do the self test of the MPU6050

	//4. turn on the acceleromter of the sensor

	//5. get the accelerometer values of the sensor

	//6. display the value of the acclerometer

	//PB6: SCL AF4 I2C1
	//PB7: SDA AF4 I2C2

	lcd16x02_RW_pin(GPIOD, GPIO_PIN_1);
	lcd16x02_RS_pin(GPIOD, GPIO_PIN_0);
	lcd16x02_Enable_pin(GPIOD, GPIO_PIN_2);

	lcd16x02_Data_pin0(GPIOE, GPIO_PIN_11);
	lcd16x02_Data_pin1(GPIOE, GPIO_PIN_12);
	lcd16x02_Data_pin2(GPIOE, GPIO_PIN_13);
	lcd16x02_Data_pin3(GPIOE, GPIO_PIN_14);

	lcd16x02_Data_pin4(GPIOD, GPIO_PIN_3);
	lcd16x02_Data_pin5(GPIOD, GPIO_PIN_4);
	lcd16x02_Data_pin6(GPIOD, GPIO_PIN_5);
	lcd16x02_Data_pin7(GPIOD, GPIO_PIN_6);

	LCD_config_t LCD_int;

	LCD_int.Blinking = LCD16x02_CURSOR_BLINK_ON;
	LCD_int.IncOrDri = LCD16x02_INCREMENT_ADDRESS;
	LCD_int.LCD_Lines = LCD16x02_2LIN_MODE;
	LCD_int.LCD_cursor = LCD16x02_CURSOR_ON;
	LCD_int.LCD_font = LCD16x02_FONT_5x8D;
	//LCD_int.LCD_mode = LCD16x02_MODE_4BITS;
	LCD_int.LCD_mode = LCD16x02_MODE_8BITS;
	LCD_int.display = LCD16x02_DISPLAY_ON;

	lcd16x02_int(&LCD_int);
	lcd16x02_clear();
	lcd16x02_setCursor(1, 4);

	lcd16x02_write_string("Hello World");

	lcd16x02_setCursor(2, 1);
	lcd16x02_write_string("Hi!! KAMAL ");
	lcd16x02_write_char(68);

	lcd16x02_blink_off();
	lcd16x02_cursor_off();


	//=======================================================

	I2C_Handle_t i2cForMPU60x0 ;
	i2cForMPU60x0.pI2Cx = I2C1;
	i2cForMPU60x0.I2C_Config.I2C_AckControl = ENABLE;
	i2cForMPU60x0.I2C_Config.I2C_DeviceAddress = 0;
	i2cForMPU60x0.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	i2cForMPU60x0.I2C_Config.I2C_SCLSpeed = 100000;
	i2cForMPU60x0.I2C_Config.slaveADDRmode = I2C_SLAVE_ADDR_MODE_7BITS;

	I2C_PeriClockControl(I2C1, ENABLE);
	I2C_Init(&i2cForMPU60x0);
	System_I2C_ForMPU = &i2cForMPU60x0;

	GPIO_Handle_t gpioForMPU;
	gpioForMPU.pGPIOx = GPIOB;
	gpioForMPU.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	gpioForMPU.GPIO_PinConfig.GPIO_PinAlFunMode = 4 ;
	gpioForMPU.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioForMPU.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioForMPU.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioForMPU.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6; // for I2C1 SCl

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&gpioForMPU);

	gpioForMPU.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7; //for I2C1 SDA

	GPIO_Init(&gpioForMPU);


	GPIO_Handle_t GpioAbtn;
	GpioAbtn.pGPIOx = GPIOA;
	GpioAbtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioAbtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioAbtn);


	MPU6050_AccelHandle_Struct_t AccelConfig ;
	AccelConfig.DLF_Val = 1 ;
	AccelConfig.FIFO_EnOrDi = DISABLE;
	AccelConfig.FS_Val = MPU6050_ACCEL_FS_4	 ;
	AccelConfig.Motion_thershold = 20;
	MPU6050_Set_ADDR(0x68);

	I2C_PeripheralControl(I2C1, ENABLE);

	MPU6050_Sleep_disable();
	lcd16x02_clear();


	MPU6050_Accel_Config(&AccelConfig);

	uint16_t Xaccel , Yaccel , Zaccel;
	while(1)
	{
		MPU6050_Get_AccelData(&Xaccel, &Yaccel, &Zaccel);

		lcd16x02_setCursor(0, 0);
		lcd16x02_write_string((uint8_t)Xaccel);
		 while(! ( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) ) );
	}


	//=============================================================================





	for(;;);
}
