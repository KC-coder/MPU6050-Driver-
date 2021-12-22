/*
 * lcd.c
 *
 *  Created on: Aug 12, 2021
 *      Author: kamal chopra
 */

#include "lcd.h"

LCD16x0Pins_info_t pinDetails;

static void write4_Bits(uint8_t value);
static void write8_Bits(uint8_t value);

static uint8_t EntryMode_val = 0 ;
static uint8_t DisplayControl_val = 0;
static uint8_t CursorDisplayShift_val = 0;
static uint8_t FunctionSet_val = 0 ;

static void Delay(uint8_t value , uint8_t MicroOrMilli);


static void Delay(uint8_t value , uint8_t MicroOrMilli)
{

	//FOR SOFTWARE BASED DELAY
	if( MicroOrMilli == MICRO_SECONDS)
	{
		for(uint32_t i=0 ; i < (value * 1); i++);

	}else
	{
		for(uint32_t i=0 ; i < (value * 1000); i++);
	}

	//FOR TIMER BASED DELAY

	/*
	 * to produce Timer delay
	 * we need 4 things
	 * 1) timer pheriheral CLK value
	 * 2) timer prescalar value
	 * 3) timer ARR value
	 * 4
	 * ask user if he wants to use TIM6 or TIM7 peripheral for producing the delays
	 *
	 *
	 * SETPS
	 * 1) take the value dealy required
	 * 2) read/find the current value of the timer peripheral clk
	 * 3) calulate the value of ARR and prescalar needed to get the required delay
	 *
	 */

/*
	TIM_Handle_t DelayTimer;

	 DelayTimer.pTIMx = LCD16x02_DELAY_TIMER ;
	 DelayTimer.TIMConfig.CounterMode = TIM_COUNTERMODE_UP;
	 DelayTimer.TIMConfig.Period = ARR;
	 DelayTimer.TIMConfig.Prescaler = PRE;

	 TIM_BASE_Init(&DelayTimer);
	 TIM_COUNTER_EnOrDi(LCD16x02_DELAY_TIMER, ENABLE);

	 while(! (LCD16x02_DELAY_TIMER->SR & (1 << TIMx_SR_UIF) ));

	 LCD16x02_DELAY_TIMER->SR &= ~(1 << TIMx_SR_UIF);*/

}


/*	initializing by internal reset circuit
 *  1) display clear
 *  2) function set	DL bit = 1 ,N bit = 0 , F bit = 0
 *  3) Display on/off control D = 0 ,C = 0 ,B =0
 *  entry mode set I/D = 1, S =0
 *  during this BF = 1 fpr 10ms after Vcc rises to 4.5
 *
 */

/*
 * NOTE : -
 *
 * When an instruction is being executed for internal operation, no instruction other
 * than the busy flag/address read instruction can be executed.
 * Because the busy flag is set to 1 while an instruction is being executed, check
 * it to make sure it is 0 before sending another instruction from the MPU.
 */

/*
 *
 * RS : Selects registers.
 *		0: Instruction register (for write) Busy flag:
 *		address counter (for read)
 *		1: Data register (for write and read)
 *
 * R/W : Selects read or write.
 * 		 0: Write
 * 		 1: Read
 *
 *
 * Be sure the HD44780U is not in the busy state (BF = 0) before sending
 * an instruction from the MPU to the HD44780U. If an instruction is sent without
 * checking the busy flag, the time between the first instruction and next instruction
 * will take much longer than the instruction time itself
 *
 * Instructions :
 * (X = don't care )
 * 					RS	R/W` code		Execution time
 * clear display :	0	0	00000001
 * Return Home	 :	0	0	0000001x	 1.52ms
 * Entry mode set:	0	0	000001 I/D S 37us			I/D = Sets cursor move direction,
 * 														S = specifies display shift
 * Display on/off:	0	0	00001DCB	 37us			D = Sets entire display on/off,
 * control												C = cursor on/off
 * 														B = blinking of cursor position character
 * Function	 : 		0 	0	001 DL N F XX	37us		DL = Sets interface data length
 * set													N  = number of display lines
 * 														F  = character font
 * Cursor or		0	0	0001	S/C R/L XX			S/C = Moves cursor
	display
	shift
 *
 * S/C R/L
 *	0 	0 	Shifts the cursor position to the left. (AC is decremented by one.)
 *	0 	1 	Shifts the cursor position to the right. (AC is incremented by one.)
 *	1 	0 	Shifts the entire display to the left. The cursor follows the display shift.
 *	1 	1 	Shifts the entire display to the right. The cursor follows the display shift.
 *
 *
 */

/* 1) Power Supply on
 * 2) function set (select 4/8 bit mode , line 1/2 , dot mode )
 * 3) display on/off control (on/off display and cursor )
 * 4) entry mode ( I/D , shift )
 * 5) write data
 * 6) set cursor
 * 7) write data
 *
 *
 *
 *
 */

/*
 * explore these things
 * 	Cursor or Display Shift instruction (pg:27 )
 *
 */
void lcd16x02_int(LCD_config_t *pLCD)
{
	//delay of 40ms
	Delay(50, MILLI_SECONDS);

	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);

	//wait for 40ms before starting as per datasheet


	write8_Bits(0X30);
	//delay of 4.1ms
	Delay(6, MILLI_SECONDS);

	write8_Bits(0X30);

	//delay for more than 100us
	Delay(150, MICRO_SECONDS);

	write8_Bits(0X30);

	#if LCD16x02_MODE_4BIT
	/*
	 * AS per the datasheet when we are using the 4 Bit mode
	 * we have to first send 0x20 because before this LCD was operating in 8 data line mode
	 * so we first select 4 bit mode by sending function set command
	 * after this we can configure reset of the LCD
	 *
	 * NOTE : after this the mode of LCD can not be changed again
	 * also after this we send function set command again to select number of lines and font
	 * and after that also the lines and font ca/t be changed of operation cannot be changed
	 */
		write8_Bits(0X20);
	#else
		/*
		 * no need to send function set command 2 times in 8 data line mode because in this mode we send
		 * only 1 time in this we send the mode ,lines, font information 1 one go
		 */
	#endif
	// uptill here is same for the 4bit and 8th mode
	//now we have to configure according to the data mode

	//after this we have to configure
	//now we can check the BF

//===================================================================

		//configuring for function set command

		FunctionSet_val |= (pLCD->LCD_mode << 4);
		FunctionSet_val |= (pLCD->LCD_font << 2);
		FunctionSet_val |= (pLCD->LCD_Lines << 3);
		//NOTE : The number of display lines and character font cannot be changed after this point.

		lcd16x02_send_cmd( LCD16x02_FUNCTION_SET_CMD | FunctionSet_val);
		//display on/off control command

		DisplayControl_val |=(1 << 2); // display on
		DisplayControl_val |=(pLCD->LCD_cursor << 1);
		DisplayControl_val |=(pLCD->Blinking << 0);

		lcd16x02_send_cmd(DisplayControl_val | LCD16x02_DISPLAY_ONOFF_CMD );

		//display clear command
		lcd16x02_send_cmd(LCD16x02_CLEAR_CMD);


		//entry mode command
		EntryMode_val |= (pLCD->IncOrDri << 1);

		lcd16x02_send_cmd(EntryMode_val | LCD16x02_ENTRYMODE_SET_CMD);
		//initialization ends now

		//=========================================================================

}


void lcd16x02_blink_on()
{
	DisplayControl_val |= (1 << 0);
	lcd16x02_send_cmd(LCD16x02_DISPLAY_ONOFF_CMD | DisplayControl_val );
}
void lcd16x02_blink_off()
{
	DisplayControl_val &= ~(1 << 0);
	lcd16x02_send_cmd(LCD16x02_DISPLAY_ONOFF_CMD | DisplayControl_val );
}
void lcd16x02_cursor_on()
{
	DisplayControl_val |= (1 << 1);
	lcd16x02_send_cmd(LCD16x02_DISPLAY_ONOFF_CMD | DisplayControl_val );
}
void lcd16x02_cursor_off()
{
	DisplayControl_val &= ~(1 << 1);
	lcd16x02_send_cmd(LCD16x02_DISPLAY_ONOFF_CMD | DisplayControl_val );
}
void lcd16x02_display_on()
{
	DisplayControl_val |= (1 << 2);
	lcd16x02_send_cmd(LCD16x02_DISPLAY_ONOFF_CMD | DisplayControl_val );
}
void lcd16x02_display_off()
{
	DisplayControl_val &= ~(1 << 2);
	lcd16x02_send_cmd(LCD16x02_DISPLAY_ONOFF_CMD | DisplayControl_val );
}
void lcd16x02_scroll_display(uint8_t leftOrRight)
{
	if(leftOrRight == SCROLL_LEFT )
	{
		CursorDisplayShift_val |= (1 << 2 );
	}else
	{
		CursorDisplayShift_val |= (1 << 2 );
	}

	CursorDisplayShift_val |= (1 << 3); // S/C BIT TO select cursor to shift
	lcd16x02_send_cmd(LCD16x02_CURSOR_ORSHIFT_CMD | CursorDisplayShift_val );
}

void lcd16x02_shift_RtoL()
{
	CursorDisplayShift_val &= ~(1 << 2);
	CursorDisplayShift_val &= ~(1 << 3); // S/C BIT TO select cursor to shift
	lcd16x02_send_cmd(LCD16x02_CURSOR_ORSHIFT_CMD	| CursorDisplayShift_val );
}
void lcd16x02_shift_LtoR()
{
	CursorDisplayShift_val |= (1 << 2 );
	CursorDisplayShift_val &= ~(1 << 3); // S/C BIT TO select cursor to shift
	lcd16x02_send_cmd(LCD16x02_CURSOR_ORSHIFT_CMD	| CursorDisplayShift_val );
}


void lcd16x02_reset()
{

}

void lcd16x02_autoscroll();
void lcd16x02_noautoscroll();




void lcd16x02_write_char(uint8_t charater)
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);
	#if LCD16x02_MODE_4BIT
		write4_Bits(charater >> 4 );
		write4_Bits(charater & 0x0F);
	#else
		write8_Bits(charater);
	#endif
}
void lcd16x02_write_string(char *message)
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);

	do
	{
		#if LCD16x02_MODE_4BIT
			write4_Bits((uint8_t ) *message >> 4 );
			write4_Bits((uint8_t ) *message & 0x0F);
		#else
			write8_Bits((uint8_t ) *message ) ;
		#endif

			++message ;

	}while(*message != '\0');

}


void lcd16x02_clear()
{

	lcd16x02_send_cmd(LCD16x02_CLEAR_CMD);

	Delay(3, MILLI_SECONDS);
	//delay 2 milli seconds


}

void lcd16x02_setCursor(uint8_t row , uint8_t column)
{
	//user writes column form 1 to 16 but the address of column
	// is from 00 to 0F
	column-- ;
	 switch (row)
	 {
	   case 1:
	      /* Set cursor to 1st row address and add index*/
		   lcd16x02_send_cmd((column |= 0x80));
	    break;
	   case 2:
	      /* Set cursor to 2nd row address and add index*/
		   lcd16x02_send_cmd((column |= 0xC0));
	     break;
	 }
}
void lcd16x02_send_cmd(uint8_t value )
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);

	#if LCD16x02_MODE_4BIT
		write4_Bits(value >> 4 );
		write4_Bits(value  & 0x0F);
	#else
		write8_Bits(value);
	#endif
		//Delay(20, MICRO_SECONDS);
}
void lcd16x02_returnHome()
{
	lcd16x02_send_cmd(LCD16X02_RETURNHOME_CMD);

}

void lcd16x02_set_DDRAM_addr(uint8_t address)
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);

	#if LCD16x02_MODE_4BIT
		write4_Bits(address >> 4 | 0x8);
		write4_Bits(address   & 0x0F);
	#else
		write8_Bits(address | 0x80);
	#endif
}
void lcd16x02_set_CGRAM_addr(uint8_t address)
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);
	//as per command to set address Bit 7 should be  0 and Bit 6 should be  1
	//and the remaing bits are for address

	address &= ~(0x80);
	address |= (0x40);

	#if LCD16x02_MODE_4BIT

		write4_Bits(address >> 4);
		write4_Bits(address  & 0x0F);
	#else

		write8_Bits(address );
	#endif
}


void lcd16x02_enable()
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.EnablePin, GPIO_PIN_SET);
	//delay of 10us
	Delay(10, MILLI_SECONDS);
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.EnablePin, GPIO_PIN_RESET);
	//delay of 10us
	Delay(100, MILLI_SECONDS);

}

/*<=====================================================================================>*/

/*
 *  Helper functions to write 4 or 8 bits
 */
static void write4_Bits(uint8_t value)
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_data4pin, pinDetails.data4pin, ((value >> 0 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data5pin, pinDetails.data5pin, ((value >> 1 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data6pin, pinDetails.data6pin, ((value >> 2 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data7pin, pinDetails.data7pin, ((value >> 3 ) & 0x01 ));

	lcd16x02_enable();
}

static void write8_Bits(uint8_t value)
{
	GPIO_WriteToOutputPin(pinDetails.Portfor_data0pin, pinDetails.data0pin, ((value >> 0 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data1pin, pinDetails.data1pin, ((value >> 1 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data2pin, pinDetails.data2pin, ((value >> 2 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data3pin, pinDetails.data3pin, ((value >> 3 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data4pin, pinDetails.data4pin, ((value >> 4 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data5pin, pinDetails.data5pin, ((value >> 5 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data6pin, pinDetails.data6pin, ((value >> 6 ) & 0x01 ));
	GPIO_WriteToOutputPin(pinDetails.Portfor_data7pin, pinDetails.data7pin, ((value >> 7 ) & 0x01 ));

	lcd16x02_enable();
}

/*<=====================================================================================>*/
/*
 * Functions to configure the GPIOs
 */

//functions to configure  the pins of our LCD16x02
void lcd16x02_RS_pin(GPIO_RegDef_t *gpioPort , uint8_t gpioPin)
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = gpioPin;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_RSpin = gpioPort;
	pinDetails.RSpin = gpioPin;
	GPIO_WriteToOutputPin(pinDetails.Portfor_RSpin, pinDetails.RSpin, GPIO_PIN_RESET);



}
void lcd16x02_RW_pin(GPIO_RegDef_t *gpioPort , uint8_t gpioPin)
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = gpioPin;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_RWpin = gpioPort;
	pinDetails.RWpin = gpioPin;
	GPIO_WriteToOutputPin(pinDetails.Portfor_RWpin, pinDetails.RWpin, GPIO_PIN_RESET);
}
void lcd16x02_Enable_pin(GPIO_RegDef_t *gpioPort , uint8_t gpioPin)
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = gpioPin;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_EnablePin = gpioPort;
	pinDetails.EnablePin = gpioPin;

	GPIO_WriteToOutputPin(pinDetails.Portfor_EnablePin, pinDetails.EnablePin, GPIO_PIN_RESET);
}

void lcd16x02_Data_pin0 (GPIO_RegDef_t *gpioPort , uint8_t datapin0 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin0;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data0pin = gpioPort;
	pinDetails.data0pin = datapin0;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data0pin, pinDetails.data0pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin1 (GPIO_RegDef_t *gpioPort , uint8_t datapin1 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin1;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data1pin = gpioPort;
	pinDetails.data1pin = datapin1;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data1pin, pinDetails.data1pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin2 (GPIO_RegDef_t *gpioPort , uint8_t datapin2 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin2;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data2pin = gpioPort;
	pinDetails.data2pin = datapin2;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data2pin, pinDetails.data2pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin3 (GPIO_RegDef_t *gpioPort , uint8_t datapin3 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin3;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data3pin = gpioPort;
	pinDetails.data3pin = datapin3;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data3pin, pinDetails.data3pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin4 (GPIO_RegDef_t *gpioPort , uint8_t datapin4 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin4;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data4pin = gpioPort;
	pinDetails.data4pin = datapin4;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data4pin, pinDetails.data4pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin5 (GPIO_RegDef_t *gpioPort , uint8_t datapin5 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin5;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data5pin = gpioPort;
	pinDetails.data5pin = datapin5;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data5pin, pinDetails.data5pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin6 (GPIO_RegDef_t *gpioPort , uint8_t datapin6 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin6;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);

	pinDetails.Portfor_data6pin = gpioPort;
	pinDetails.data6pin = datapin6;
	GPIO_WriteToOutputPin(pinDetails.Portfor_data6pin, pinDetails.data6pin, GPIO_PIN_RESET);
}
void lcd16x02_Data_pin7 (GPIO_RegDef_t *gpioPort , uint8_t datapin7 )
{
	GPIO_Handle_t pin;
	pin.pGPIOx = gpioPort;
	pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinNumber = datapin7;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(gpioPort, ENABLE);
	GPIO_Init(&pin);
	pinDetails.Portfor_data7pin = gpioPort;
	pinDetails.data7pin = datapin7;

	GPIO_WriteToOutputPin(pinDetails.Portfor_data7pin, pinDetails.data7pin, GPIO_PIN_RESET);
}

