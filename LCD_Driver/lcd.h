/*
 * lcd.h
 *
 *  Created on: Aug 12, 2021
 *
 *  Updated on :Aug 18, 2021
 *
 *      Author: Kamal Chopra
 */

#ifndef LCD_H_
#define LCD_H_


#include "stm32f407xx.h"
#include "stm32F407xx_gpio_driver.h"


#define LCD16x02_MODE_4BIT			0 /*<To select the operating mode of LCD that is we want 4 dataLines or 8 dataLines>
 	 	 	 	 	 	 	 	 	 	 	If it is 0 then LCD is operating in 8 data lines mode
 	 	 	 	 	 	 	 	 	 	 	if it is 1 then LCD is operating in 4 data lines mode*/
#define LCD16x02_DELAY_TIMER_BASED	1/*<To select if the delays required in the driver should software based or timer based
 	 	 	 	 	 	 	 	 	 		If it is 1 then TIMER based delay is used
 	 	 	 	 	 	 	 	 	 		If it is 0 then SOFTWARE based delay is used
  	  	  	  	  	  	  	  	  	  	  	NOTE : Software delays are based on 16MHz system clock
  	  	  	  	  	  	  	  	  	  	  	NOTE : For timer delays can be used for any timer frequency because
  	  	  	  	  	  	  	  	  	  	  		in those delay we read the system clk val and based on that make timer count
  	  	  	  	  	  	  	  	  	  	  	NOTE : FOR using the timer delay we have to Specify the timer we are using
  	  	  	  	  	  	  	  	  	  	  	 	 in the LCD16x02_DELAY_TIMER macro >*/


#if LCD16x02_DELAY_TIMER_BASED

#include "stm32F407xx_TIM_driver.h"
#include "stm32F407xx_RCC_driver.h"
#define LCD16x02_DELAY_TIMER 		TIM6


#endif



typedef struct
{
	uint8_t RWpin;
	GPIO_RegDef_t *Portfor_RWpin;
	uint8_t RSpin;
	GPIO_RegDef_t *Portfor_RSpin;
	uint8_t EnablePin;
	GPIO_RegDef_t *Portfor_EnablePin;
	uint8_t data0pin;
	GPIO_RegDef_t *Portfor_data0pin;
	uint8_t data1pin;
	GPIO_RegDef_t *Portfor_data1pin;
	uint8_t data2pin;
	GPIO_RegDef_t *Portfor_data2pin;
	uint8_t data3pin;
	GPIO_RegDef_t *Portfor_data3pin;
	uint8_t data4pin;
	GPIO_RegDef_t *Portfor_data4pin;
	uint8_t data5pin;
	GPIO_RegDef_t *Portfor_data5pin;
	uint8_t data6pin;
	GPIO_RegDef_t *Portfor_data6pin;
	uint8_t data7pin;
	GPIO_RegDef_t *Portfor_data7pin;

}LCD16x0Pins_info_t;


typedef struct
{
	uint8_t LCD_mode; // 4bit or 8bit
	uint8_t LCD_font; // 5x8 or 8x10
	uint8_t LCD_cursor; // cursor on or off
	uint8_t display; // on or off
	uint8_t LCD_Lines; // 1 line or 2 lines of display
	uint8_t Blinking; // on/off
	uint8_t shift; // right of left
	uint8_t IncOrDri ;
}LCD_config_t;

/*
 * commands/instruction codes of the LCD
 */

#define LCD16x02_CLEAR_CMD				0x01
#define LCD16X02_RETURNHOME_CMD			0x02
#define LCD16x02_ENTRYMODE_SET_CMD		0x04
#define LCD16x02_DISPLAY_ONOFF_CMD		0x08
#define LCD16x02_CURSOR_ORSHIFT_CMD		0x10
#define LCD16x02_FUNCTION_SET_CMD		0x20
#define LCD16x02_SET_CGRAM_ADDR_CMD		0x40
#define LCD16x02_SET_DDRAM_ADDR_CMD		0x80



/* For the address increment or Decrement
 * we have a option i.e after reading /writing ON
 * CG or DDRAM we want to increment or Decrement address of
 * address counter
 * used to configure the I/D bit of Entry mode set command
 */
#define LCD16x02_INCREMENT_ADDRESS		1
#define LCD16x02_DECREMENT_ADDRESS		0


/* FOR TURING ON/OFF THE DISPLAY
 * used to configure the D bit of Display on/off control command
 */
#define LCD16x02_DISPLAY_ON		1
#define LCD16x02_DISPLAY_OFF	0

/* FOR TURING ON/OFF THE  cursor of the display
 * used to configure the C bit of Display on/off control command
 */
#define LCD16x02_CURSOR_ON		1
#define LCD16x02_CURSOR_OFF		0

/*
 * FOR TURING ON/OFF CURSOR BLINKING
 * used to configure the B bit of Display on/off control command
 */
#define LCD16x02_CURSOR_BLINK_ON	1
#define LCD16x02_CURSOR_BLINK_OFF	0

/* For Cursor/ Display shift selection
 * that is  we want to shift the cursor or the display when shift operation is performed
 * These are used in configuration  of S/C bit of cursor or display shift command
 */
#define LCD16x02_SHIFT_CURSOR		0
#define LCD16x02_SHIFT_DISPLAY		1

/*
 * For ShiftRight / ShiftLeft selection
 * shifts the cursor position or display to the right or left without writing or reading
 * display data. This function is used to correct or search the display
 *  used to configure the R/L bit of cursor or display shift command
 */
#define LCD16x02_SHIFT_RIGHT		1
#define LCD16x02_SHIFT_LEFT			0

/* For selecting the data mode of LCD
 * i.e we want to work with 4 data lines of LCD or 8 data lines
 * Used   to configure the DL bit of Function set instruction
 *
 */
#define LCD16x02_MODE_4BITS			0
#define LCD16x02_MODE_8BITS			1

/* For selecting the font we want
 * by default we have 2 types of founts 5x10 and 5x8
 * Used to  configure the  F bit of function set instruction
 */

#define LCD16x02_FONT_5x10D			1 // 5x10 dots
#define LCD16x02_FONT_5x8D			0 // 5x8 dots

/* For  selecting the number of lines off the display
 * the display can be 1 line or 2 line
 * used to configure the N bit of function set instruction
 */
#define LCD16x02_2LIN_MODE			1
#define LCD16x02_1LINE_MODE			0




/* General Macros*/

#define SCROLL_LEFT 						0
#define SCROLL_RIGHT						1
#define MICRO_SECONDS						2
#define MILLI_SECONDS						3

/*<APIs for the LCD driver >*/

//API for the LCD PIN Config
void lcd16x02_RS_pin(GPIO_RegDef_t *gpioPort , uint8_t gpioPin);
void lcd16x02_RW_pin(GPIO_RegDef_t *gpioPort , uint8_t gpioPin);
void lcd16x02_Enable_pin(GPIO_RegDef_t *gpioPort , uint8_t gpioPin);

void lcd16x02_Data_pin0 (GPIO_RegDef_t *gpioPort , uint8_t datapin0 );
void lcd16x02_Data_pin1 (GPIO_RegDef_t *gpioPort , uint8_t datapin1 );
void lcd16x02_Data_pin2 (GPIO_RegDef_t *gpioPort , uint8_t datapin2 );
void lcd16x02_Data_pin3 (GPIO_RegDef_t *gpioPort , uint8_t datapin3 );
void lcd16x02_Data_pin4 (GPIO_RegDef_t *gpioPort , uint8_t datapin4 );
void lcd16x02_Data_pin5 (GPIO_RegDef_t *gpioPort , uint8_t datapin5 );
void lcd16x02_Data_pin6 (GPIO_RegDef_t *gpioPort , uint8_t datapin6 );
void lcd16x02_Data_pin7 (GPIO_RegDef_t *gpioPort , uint8_t datapin7 );




void lcd16x02_int();
void lcd16x02_clear();
void lcd16x02_write_char(uint8_t data);
void lcd16x02_write_string(char *message);
void lcd16x02_setCursor(uint8_t row , uint8_t column);
void lcd16x02_send_cmd(uint8_t value);
void lcd16x02_returnHome();

void lcd16x02_blink_on();
void lcd16x02_blink_off();
void lcd16x02_cursor_on();
void lcd16x02_cursor_off();
void lcd16x02_display_on();
void lcd16x02_display_off();
void lcd16x02_shift_RtoL();
void lcd16x02_shift_LtoR();
void lcd16x02_scroll_display(uint8_t leftOrRight);


void lcd16x02_returnHome();

void lcd16x02_set_DDRAM_addr(uint8_t address);
void lcd16x02_set_CGRAM_addr(uint8_t address);
void lcd16x02_enable();

//TODO
void lcd16x02_autoscroll();
void lcd16x02_noautoscroll();

void lcd16x02_reset();
//=====================




#endif /* LCD_H_ */
