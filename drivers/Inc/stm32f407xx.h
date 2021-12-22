/*
 * stm32f407xx.h
 *
 *  Created on: Aug 20, 2020
 *      Author: kamal chopra
 *      part of MCU-1
 */

/* We can use lower case for C macros , but prefer to use
 * upper case as it is the style of embedded programming
 * we can use prefix/suffix DRV_ to differentiate macro
 * from code as in bigger projects their are lots of code*/

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h> //as we are using its contents for structures
/* for base address of memories*/
#include<stddef.h>

/*=============================PROCESSOR SPECIFIC DETAILS=======================================*/
/*============================FOR ARM CORTEX-M4 PROCESSOR=========================================*/

/*
 *  processor NVIC ISERx register addresses
 *
 * address has to be type casted to volatile uint32_t* else we can't dereference
 * the value at the value at the address
 */

#define NVIC_ISER0						((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1						((volatile uint32_t*) 0xE000E100 + 0x01 )
#define NVIC_ISER2						((volatile uint32_t*) 0xE000E100 + 0x02 )
#define NVIC_ISER3						((volatile uint32_t*) 0xE000E100 + 0x03 )

/*
 *  processor NVIC ICERx register addresses
 *
 * address has to be type casted to volatile uint32_t* else we can't dereference
 * the value at the value at the address
 */

#define NVIC_ICER0						 ((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1						 ((volatile uint32_t*) 0xE000E180 + 0x04 )
#define NVIC_ICER2						 ((volatile uint32_t*) 0xE000E180 + 0x08 )
#define NVIC_ICER3						 ((volatile uint32_t*) 0xE000E180 + 0x0c )

/*
 * processor NVIC NVIC_IPR register addresses
 */

#define NVIC_PR_BASE_ADDR				 ((volatile uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED				4

/*===============================================================================================*/
#define	FLASH_BASEADDR							0x08000000U		//base address for flash
#define SRAM1_BASEADDR							0x20000000U		//base address of main sram
#define SRAM2_BASEADDR							0X2001C000U		//SRM1 is 112kb to add this in base to get arress of srm2
#define ROM										0x1FFF0000U
#define SYSTEM_MEMORY_BASEADDR					ROM
#define SRAM									SRAM1_BASEADDR	//because sram1 is the main sram

/*for base address of  AHBX and APBX bus  peripheral */

#define PERIPH_BASEADDR							0x40000000U
#define APB1PERIPH_BASE							PERIPH_BASEADDR
#define APB2PERIPH_BASE							0x40010000U
#define AHB1PERIPH_BASE							0x40020000U
#define AHB2PERIPH_BASE							0x50000000U
#define AHB3PERIPH_BASE							0xA0000000U


/* base addresses of peripherals which are hanging on AHB1 bus */
																/*OFFSET*/
#define GPIOA_BASEADDR							(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR							(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR							(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR							(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR							(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR							(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR							(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR							(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR							(AHB1PERIPH_BASE + 0x2000)
#define CRC_BASEADDR							(AHB1PERIPH_BASE + 0x3000)
#define RCC_BASEADDR							(AHB1PERIPH_BASE + 0x3800)

#define RCC										((RCC_RegDef_t*)RCC_BASEADDR)


/* type casting GPIO's base address to GPIO_RegDef_t type*/
#define GPIOA									((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB									((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC									((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD									((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE									((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF									((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG									((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH									((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI									((GPIO_RegDef_t*)GPIOI_BASEADDR)

/* base addresses of peripherals which are hanging on APB1 bus
 * (changing style of writing )					*/
																/*OFFSET*/

#define TIM2_BASEADDR							0x40000000U
#define TIM3_BASEADDR							0x40000400U
#define TIM4_BASEADDR							0x40000800U
#define TIM5_BASEADDR							0x40000C00U
#define TIM6_BASEADDR							0x40001000U
#define TIM7_BASEADDR							0x40001400U
#define TIM12_BASEADDR							0x40001800U
#define TIM13_BASEADDR							0x40001C00U
#define TIM14_BASEADDR							0x40002000U

#define SPI2_BASEADDR							0x40003800U
#define SPI3_BASEADDR							0x40003C00U

#define USART2_BASEADDR							0x40004400U
#define USART3_BASEADDR							0x40004800U

#define UART4_BASEADDR							0x40004C00U
#define UART5_BASEADDR							0x40005000U
#define UART7_BASEADDR							0x40007800U
#define UART8_BASEADDR							0x40007C00U

#define I2C1_BASEADDR							0x40005400U
#define I2C2_BASEADDR							0x40005800U
#define I2C3_BASEADDR							0x40005C00U

#define CAN1_BASEADDR							0x40006400U
#define CAN2_BASEADDR							0x40006800U
#define PWR_BASEADDR							0x40007000U
#define DAC_BASEADDR							0x40007400U



/* base addresses of peripherals which are hanging on APB2 bus
 * (changing style of writing )					*/
																/*OFFSET*/

#define TIM1_BASEADDR							0x40010000U
#define TIM8_BASEADDR							0x40010400U
#define USART1_BASEADDR							0x40011000U
#define USART6_BASEADDR							0x40011400U
#define SPI1_BASEADDR							0x40013000U
#define SPI4_BASEADDR							0x40013400U
#define SYSCFG_BASEADDR							0x40013800U
#define EXTI_BASEADDR							0x40013C00U
#define TIM9_BASEADDR							0x40014000U
#define TIM10_BASEADDR							0x40014400U
#define TIM11_BASEADDR							0x40014800U


#define EXTI									((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG									((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define ADC1_BASEADDR							0x40012000U
#define ADC2_BASEADDR							0x40012100U
#define ADC3_BASEADDR							0x40012200U
#define ADC_CommonReg_BASEADDR					0x40012300U

/* type casting SPIx's base address to SPI_RegDef_t type*/

#define SPI1									((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2									((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3									((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4									((SPI_RegDef_t*)SPI4_BASEADDR)

/* type casting I2Cx's base address to I2C_RegDef_t*/

#define I2C1									((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2									((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3									((I2C_RegDef_t*)I2C3_BASEADDR)


/* type casting USARTx's base address to USART_RegDef_t type*/

#define USART1									((USART_RegDef_t*)USART1_BASEADDR)
#define USART2									((USART_RegDef_t*)USART2_BASEADDR)
#define USART3									((USART_RegDef_t*)USART3_BASEADDR)
#define UART4									((USART_RegDef_t*)UART4_BASEADDR)
#define UART5									((USART_RegDef_t*)UART5_BASEADDR)
#define USART6									((USART_RegDef_t*)USART6_BASEADDR)
#define UART7									((USART_RegDef_t*)UART7_BASEADDR)
#define UART8									((USART_RegDef_t*)UART8_BASEADDR)

/* type casting TIMx's base address to TIM_RegDef_t type*/
#define TIM1									((TIM_RegDef_t*)TIM1_BASEADDR)
#define TIM2									((TIM_RegDef_t*)TIM2_BASEADDR)
#define TIM3									((TIM_RegDef_t*)TIM3_BASEADDR)
#define TIM4									((TIM_RegDef_t*)TIM4_BASEADDR)
#define TIM5									((TIM_RegDef_t*)TIM5_BASEADDR)
#define TIM6									((TIM_RegDef_t*)TIM6_BASEADDR)
#define TIM7									((TIM_RegDef_t*)TIM7_BASEADDR)
#define TIM8									((TIM_RegDef_t*)TIM8_BASEADDR)
#define TIM9									((TIM_RegDef_t*)TIM9_BASEADDR)
#define TIM10									((TIM_RegDef_t*)TIM10_BASEADDR)
#define TIM11									((TIM_RegDef_t*)TIM11_BASEADDR)
#define TIM12									((TIM_RegDef_t*)TIM12_BASEADDR)
#define TIM13									((TIM_RegDef_t*)TIM13_BASEADDR)
#define TIM14									((TIM_RegDef_t*)TIM14_BASEADDR)

/* type casting ADCx's base address to ADC_RegDef_t type*/
#define ADC1									((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2									((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3									((ADC_RegDef_t*)ADC3_BASEADDR)
#define ADC_ComReg							((ADC_CommonRegDef_t*)ADC_CommonReg_BASEADDR)



/*=============================================================================*/

/*peripheral register definition structure
 * using volatile keyword because these register are highly volatile in nature
 *
 * HERE WE HAVE CREATED A STANDARED GPIOX SO IT CAN BE USED BY ALL THE GPIO PERIPHERAL
 * to access this create structure pointer variable and derefernce it */

typedef struct
{
		volatile uint32_t MODER;  //GPIO port mode register
		volatile uint32_t OTYPER; //GPIO port output type register
		volatile uint32_t OSPEEDR; //GPIO port output speed register
		volatile uint32_t PUPDR; //GPIO port pull-up/pull-down register
		volatile uint32_t IDR; //GPIO port input data register
		volatile uint32_t ODR; //GPIO port output data register
		volatile uint32_t BSRR; //GPIO port bit set/reset register
		volatile uint32_t LCKR; //GPIO port configuration lock register
		volatile uint32_t AFR[2]; // AFR[0] = GPIO alternate function low register; AFR[1] =GPIO alternate function high register

}GPIO_RegDef_t;


/* Peripheral register definition structure for RCC */

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
			 uint32_t Reserved1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
			 uint32_t Reserved2;
			 uint32_t Reserved3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
			 uint32_t Reserved4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t Reserved5;
	volatile uint32_t Reserved6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t Reserved7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t Reserved8;
	volatile uint32_t Reserved9;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
			 uint32_t Reserved10;
			 uint32_t Reserved11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;

}RCC_RegDef_t;


/* Peripheral register definition structure for EXTI */

typedef struct
{
	volatile uint32_t IMR;	//
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;



}EXTI_RegDef_t;

/* Peripheral register definition structure for SYSCFG */

typedef struct
{
	volatile uint32_t MEMRMP;	//SYSCFG memory remap register
	volatile uint32_t PMC;		//SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register
			 uint32_t RESERVED[2];
	volatile uint32_t CMPCR;	//Compensation cell control register



}SYSCFG_RegDef_t;


/* Peripheral register definition structure for SPI */

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;


typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;

}USART_RegDef_t;


/********************************************************************************/
/*****Peripherals related t0 MCU-2 , created solely by me ********/


typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RESERVED;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t RESERVED2;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t OR;

}TIM_RegDef_t;




typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	volatile uint32_t JOFRx[4];
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR1;
	volatile uint32_t SQR2;
	volatile uint32_t SQR3;
	volatile uint32_t JSQR;
	volatile uint32_t JDRx[4];
	volatile uint32_t DR;


}ADC_RegDef_t;

typedef struct
{
	volatile uint32_t CSR;
	volatile uint32_t CCR;
	volatile uint32_t CDR;
}ADC_CommonRegDef_t;









/*============================================================================== */

/* Macros for clock enable and disable
 * creating this as clk control is frequently required when working on peripherals */

/* clock Enable for GPIOx Peripherals */

#define GPIOA_PCLK_EN()							( RCC ->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()							( RCC ->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()							( RCC ->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()							( RCC ->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()							( RCC ->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()							( RCC ->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()							( RCC ->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()							( RCC ->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN()							( RCC ->AHB1ENR |= (1<<8) )

/* Clock disable for GPIOx Peripherals */

#define GPIOA_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI()							( RCC ->AHB1ENR &= ~(1<<8) )

/* clock Enable for I2Cx Peripherals */

#define I2C1_PCLK_EN()							( RCC ->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()							( RCC ->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()							( RCC ->APB1ENR |= (1<<23) )

/* Clock disable for I2Cx Peripherals */

#define I2C1_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<23) )

/* clock Enable for SPIx Peripherals */

#define SPI2_PCLK_EN()							( RCC ->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()							( RCC ->APB1ENR |= (1<<15) )

#define SPI1_PCLK_EN()							( RCC ->APB2ENR |= (1<<12) )
#define SPI4_PCLK_EN()							( RCC ->APB2ENR |= (1<<13) )

/* Clock disable for SPIx Peripherals */

#define SPI2_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<15) )

#define SPI1_PCLK_DI()							( RCC ->APB2ENR &= ~(1<<12) )
#define SPI4_PCLK_DI()							( RCC ->APB2ENR &= ~(1<<13) )

/* Clock Enable for USARTx Peripherals */

#define USART2_PCLK_EN()						( RCC ->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()						( RCC ->APB1ENR |= (1<<18) )
#define UART4_PCLK_EN()							( RCC ->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN()							( RCC ->APB1ENR |= (1<<20) )

#define USART1_PCLK_EN()						( RCC ->APB2ENR |= (1<<4) )
#define USART6_PCLK_EN()						( RCC ->APB2ENR |= (1<<5) )

/* Clock disable for USARTx Peripherals */

#define USART2_PCLK_DI()						( RCC ->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()						( RCC ->APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI()							( RCC ->APB1ENR &= ~(1<<20) )

#define USART1_PCLK_DI()						( RCC ->APB2ENR &= ~(1<<4) )
#define USART6_PCLK_DI()						( RCC ->APB2ENR &= ~(1<<5) )

/* Clock Enable for SYSCFG Peripherals */

#define SYSCFG_PCLK_EN()						( RCC ->APB2ENR |= (1<<14) )

/* Clock disable for SYSCFG Peripherals */

#define SYSCFG_PCLK_DN()						( RCC ->APB2ENR &= ~(1<<14))

/* Clock Enable for TIMERx Peripherals */

#define TIM1_PCLK_EN()						( RCC ->APB2ENR |= (1<<0) )
#define TIM2_PCLK_EN()						( RCC ->APB1ENR |= (1<<0) )
#define TIM3_PCLK_EN()						( RCC ->APB1ENR |= (1<<1) )
#define TIM4_PCLK_EN()						( RCC ->APB1ENR |= (1<<2) )
#define TIM5_PCLK_EN()						( RCC ->APB1ENR |= (1<<3) )
#define TIM6_PCLK_EN()						( RCC ->APB1ENR |= (1<<4) )
#define TIM7_PCLK_EN()						( RCC ->APB1ENR |= (1<<5) )
#define TIM8_PCLK_EN()						( RCC ->APB2ENR |= (1<<1) )
#define TIM9_PCLK_EN()						( RCC ->APB2ENR |= (1<<16))
#define TIM10_PCLK_EN()						( RCC ->APB2ENR |= (1<<17))
#define TIM11_PCLK_EN()						( RCC ->APB2ENR |= (1<<18))
#define TIM12_PCLK_EN()						( RCC ->APB1ENR |= (1<<6) )
#define TIM13_PCLK_EN()						( RCC ->APB1ENR |= (1<<7) )
#define TIM14_PCLK_EN()						( RCC ->APB1ENR |= (1<<8) )

/* Clock disable for TIMERx Peripherals */

#define TIM1_PCLK_DN()						( RCC ->APB2ENR &= ~(1<<0))
#define TIM2_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<0))
#define TIM3_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<1))
#define TIM4_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<2))
#define TIM5_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<3))
#define TIM6_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<4))
#define TIM7_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<5))
#define TIM8_PCLK_DN()						( RCC ->APB2ENR &= ~(1<<1))
#define TIM9_PCLK_DN()						( RCC ->APB2ENR &= ~(1<<16))
#define TIM10_PCLK_DN()						( RCC ->APB2ENR &= ~(1<<17))
#define TIM11_PCLK_DN()						( RCC ->APB2ENR &= ~(1<<18))
#define TIM12_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<6))
#define TIM13_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<7))
#define TIM14_PCLK_DN()						( RCC ->APB1ENR &= ~(1<<8))

/* Clock enable for ADCx Peripherals*/

#define ADC1_PCLK_EN						( RCC ->APB2ENR |= (1<<8) )
#define ADC2_PCLK_EN						( RCC ->APB2ENR |= (1<<9) )
#define ADC3_PCLK_EN						( RCC ->APB2ENR |= (1<<10))

/* Clock disable for ADCx Peripherals*/

#define ADC1_PCLK_DN						( RCC ->APB2ENR &= ~(1<<8) )
#define ADC2_PCLK_DN						( RCC ->APB2ENR &= ~(1<<9) )
#define ADC3_PCLK_DN						( RCC ->APB2ENR &= ~(1<<10))


/*  Macros to reset GPIOx Peripherals
 * here we have used do - while condition zero loop so we can include
 * multiple statement in a macro 										 */

#define GPIOA_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<0)); (RCC ->AHB1RSTR &= ~(1<<0));  }while(0)
#define GPIOB_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<1)); (RCC ->AHB1RSTR &= ~(1<<1));  }while(0)
#define GPIOC_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<2)); (RCC ->AHB1RSTR &= ~(1<<2));  }while(0)
#define GPIOD_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<3)); (RCC ->AHB1RSTR &= ~(1<<3));  }while(0)
#define GPIOE_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<4)); (RCC ->AHB1RSTR &= ~(1<<4));  }while(0)
#define GPIOF_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<5)); (RCC ->AHB1RSTR &= ~(1<<5));  }while(0)
#define GPIOG_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<6)); (RCC ->AHB1RSTR &= ~(1<<6));  }while(0)
#define GPIOH_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<7)); (RCC ->AHB1RSTR &= ~(1<<7));  }while(0)
#define GPIOI_REG_RESET()				do{ (RCC ->AHB1RSTR |= (1<<8)); (RCC ->AHB1RSTR &= ~(1<<8));  }while(0)


/*  Macros to reset SPIx Peripherals
 * here we have used do - while condition zero loop so we can include
 * multiple statement in a macro 										 */

#define SPI1_REG_RESET()				do{ (RCC ->APB2RSTR |= (1<<12)); (RCC ->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<14)); (RCC ->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<15)); (RCC ->APB1RSTR &= ~(1<<15)); }while(0)
#define SPI4_REG_RESET()				do{ (RCC ->APB2RSTR |= (1<<13)); (RCC ->APB2RSTR &= ~(1<<13)); }while(0)


/*  Macros to reset USARTx Peripherals
 * here we have used do - while condition zero loop so we can include
 * multiple statement in a macro 										 */



#define USART1_REG_RESET()				do{ (RCC ->APB2RSTR |= (1<<4)) ; (RCC ->APB2RSTR &= ~(1<<4)) ;  }while(0)
#define USART2_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<17)); (RCC ->APB1RSTR &= ~(1<<17));  }while(0)
#define USART3_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<18)); (RCC ->APB1RSTR &= ~(1<<18));  }while(0)
#define UART4_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<19)); (RCC ->APB1RSTR &= ~(1<<19));  }while(0)
#define UART5_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<20)); (RCC ->APB1RSTR &= ~(1<<20));  }while(0)
#define USART6_REG_RESET()				do{ (RCC ->APB1RSTR |= (1<<5)) ; (RCC ->APB1RSTR &= ~(1<<5)) ;  }while(0)





/*
 * to return port code for GPIOx base address
 * we can use function or form MACRO
 */

#define GPIO_BASEADDR_TO_CODE(x) (	(x == GPIOA)?0 :\
									(x == GPIOB)?1 :\
									(x == GPIOC)?2 :\
									(x == GPIOD)?3 :\
									(x == GPIOE)?4 :\
									(x == GPIOF)?5 :\
									(x == GPIOG)?6 :\
									(x == GPIOH)?7 :\
									(x == GPIOI)?8 :0 )


/*
 * Interrupt request number(IRQ) macros
 * IRQ number values are derived from the vector table of interrupt
 * (mentioned as position)
 */
#define IRQ_N0_EXTI0					6
#define IRQ_N0_EXTI1					7
#define IRQ_N0_EXTI2					8
#define IRQ_N0_EXTI3					9
#define IRQ_N0_EXTI4					10
#define IRQ_N0_EXTI9_5					23
#define IRQ_N0_EXTI15_10				40

#define IRQ_N0_SPI1						35
#define IRQ_N0_SPI2						36
#define IRQ_N0_SPI3						51

#define IRQ_N0_I2C1_EV					31
#define IRQ_N0_I2C1_ER					32
#define IRQ_N0_I2C2_EV					33
#define IRQ_N0_I2C2_ER					34
#define IRQ_N0_I2C3_EV					72
#define IRQ_N0_I2C3_ER					73


#define IRQ_N0_USART1					37
#define IRQ_N0_USART2					38
#define IRQ_N0_USART3					39
#define IRQ_N0_UART4					52
#define IRQ_N0_UART5					53
#define IRQ_N0_USART6					71

#define IRQ_N0_TIM1_BRK_TIM9			24	/*TIM1 Break interrupt and TIM9 global interrupt */
#define IRQ_N0_TIM1_UP_TIM10			25	/*TIM1 Update interrupt and TIM10 global interrupt */
#define IRQ_N0_TIM1_TRG_COM_TIM11		26	/*TIM1 Trigger and Commutation interrupts and TIM11 global interrupt */
#define IRQ_N0_TIM1_CC					27	/*TIM1 Capture Compare interrupt */
#define IRQ_N0_TIM2						28
#define IRQ_N0_TIM3						29
#define IRQ_N0_TIM4						30
#define IRQ_N0_TIM8_BRK_TIM12			43
#define IRQ_N0_TIM8_UP_TIM13			44
#define IRQ_N0_TIM8_TRG_COM_TIM14		45
#define IRQ_N0_TIM8_CC					46
#define IRQ_N0_TIM5						50
#define IRQ_N0_TIM6_DAC					54
#define IRQ_N0_TIM7						55



/*
 * NVIC interrupt  priority level's
 */

#define NVIC_IRQ_PRI0					0
#define NVIC_IRQ_PRI1					1
#define NVIC_IRQ_PRI2					2
#define NVIC_IRQ_PRI3					3
#define NVIC_IRQ_PRI4					4
#define NVIC_IRQ_PRI5					5
#define NVIC_IRQ_PRI6					6
#define NVIC_IRQ_PRI7					7
#define NVIC_IRQ_PRI8					8
#define NVIC_IRQ_PRI9					9
#define NVIC_IRQ_PRI10					10
#define NVIC_IRQ_PRI11					11
#define NVIC_IRQ_PRI12					12
#define NVIC_IRQ_PRI13					13
#define NVIC_IRQ_PRI14					14
#define NVIC_IRQ_PRI15					15


/*
 * some generic macros , used in various other drivers USE these
 */

#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/*======================================================================================
 *  Bit position definitions of SPI peripherals
 *=======================================================================================*/

/*
 * Bit position definition for SPI_CR1
 */

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

/*
 * Bit position definition for SPI_CR2
 */

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

/*
 * Bit position definition for SPI_SR
 */

#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8


/*======================================================================================
 *  Bit position definitions of I2Cx peripherals
 *=======================================================================================*/

/*
 * Bit position definition for I2C_CR1
 */

#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_Reserved			2
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_Reserved2			14
#define I2C_CR1_SWRST				15

/*
 * Bit position definition for I2C_CR2
 */

#define I2C_CR2_FREQ					0
#define I2C_CR2_Reserved				6
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12
#define I2C_CR2_Reserved2				13

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_PECERR 					12
#define I2C_SR1_TIMEOUT 				14
#define I2C_SR1_SMBALERT 				15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_SMBDEFAULT 				5
#define I2C_SR2_SMBHOST 				6
#define I2C_SR2_DUALF 					7
#define I2C_SR2_PEC		 				8

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/*======================================================================================
 *  Bit position definitions of USART peripherals
 *=======================================================================================*/

/*
 * Bit position definition for USART_CR1
 */

#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_Reserved			14
#define USART_CR1_OVER8				15

/*
 * Bit position definition for USART_CR2
 */

#define USART_CR2_ADD				1
#define USART_CR2_Res1				4
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_Res2				7
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14
#define USART_CR2_Res3				15

/*
 * Bit position definition for USART_CR3
 */

#define USART_CR3_EIE					0
#define USART_CR3_IREN					1
#define USART_CR3_IRLP					2
#define USART_CR3_HDSEL					3
#define USART_CR3_NACK					4
#define USART_CR3_SCEN					5
#define USART_CR3_DMAR					6
#define USART_CR3_DMAT					7
#define USART_CR3_RTSE					8
#define USART_CR3_CTSE					9
#define USART_CR3_CTSIE					10
#define USART_CR3_ONEBIT				11


/*
 * Bit position definition for USART_SR
 */

#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9


/*
 * Bit position definition for USART_SR
 */

#define USART_BRR_DIV_Fraction		0
#define USART_BRR_DIV_Mantissa		4

/*
 * Bit position definition for USART_GTPR
 */

#define USART_GTPR_DIV_PSC		0
#define USART_GTPR_DIV_GT		8





/*======================================================================================
 *  Bit position definitions of ADC peripherals
 *=======================================================================================*/

/*
 * Bit position definition for ADC_SR
 */

#define ADC_SR_AWD				0
#define ADC_SR_EOC				1
#define ADC_SR_JEOC				2
#define ADC_SR_JSTRT			3
#define ADC_SR_STRT				4
#define ADC_SR_OVR				5

/*
 * Bit position definition for ADC_CR1
 */

#define ADC_CR1_AWDCH			0
#define ADC_CR1_EOCIE			5
#define ADC_CR1_AWDIE			6
#define ADC_CR1_JEOCIE			7
#define ADC_CR1_SCAN			8
#define ADC_CR1_AWDSGL			9
#define ADC_CR1_JAUTO			10
#define ADC_CR1_DISCEN			11
#define ADC_CR1_JDISCEN			12
#define ADC_CR1_DISCNUM			13
#define ADC_CR1_Reserved1		16
#define ADC_CR1_JAWDEN			22
#define ADC_CR1_AWDEN			23
#define ADC_CR1_RES				24
#define ADC_CR1_OVRIE			26
#define ADC_CR1_Reserved2		27

/*
 * Bit position definition for ADC_CR2
 */

#define ADC_CR2_ADON			0
#define ADC_CR2_CONT			1
#define ADC_CR2_Reserved1		2
#define ADC_CR2_DMA				8
#define ADC_CR2_DDS				9
#define ADC_CR2_EOCS			10
#define ADC_CR2_ALIGN			11
#define ADC_CR2_Reserved2		12
#define ADC_CR2_JEXTSEL			16
#define ADC_CR2_JEXTEN			20
#define ADC_CR2_JSWSTART		22
#define ADC_CR2_Reserved3		23
#define ADC_CR2_EXTSEL			24
#define ADC_CR2_EXTEN			28
#define ADC_CR2_SWSTART			30
#define ADC_CR2_reserved4		31


/*
 * Bit position definition for ADC_SMPR1
 */

#define ADC_SMPR1_SMP10			0
#define ADC_SMPR1_SMP11			3
#define ADC_SMPR1_SMP12			6
#define ADC_SMPR1_SMP13			9
#define ADC_SMPR1_SMP14			12
#define ADC_SMPR1_SMP15			15
#define ADC_SMPR1_SMP16			18
#define ADC_SMPR1_SMP17			21
#define ADC_SMPR1_SMP18			24
#define ADC_SMPR1_Reserved		27

/*
 * Bit position definition for ADC_SMPR2
 */

#define ADC_SMPR2_SMP0				0
#define ADC_SMPR2_SMP1				3
#define ADC_SMPR2_SMP2				6
#define ADC_SMPR2_SMP3				9
#define ADC_SMPR2_SMP4				12
#define ADC_SMPR2_SMP5				15
#define ADC_SMPR2_SMP6				18
#define ADC_SMPR2_SMP7				21
#define ADC_SMPR2_SMP8				24
#define ADC_SMPR2_SMP9				27
#define ADC_SMPR2_Reserved			30

/*
 * Bit position definition for ADC_SQR1
 */

#define ADC_SQR1_SQ13				0
#define ADC_SQR1_SQ14				5
#define ADC_SQR1_SQ15				10
#define ADC_SQR1_SQ16				15
#define ADC_SQR1_L					20
#define ADC_SQR1_Reserved			24

/*
 * Bit position definition for ADC_SQR2
 */

#define ADC_SQR2_SQ7				0
#define ADC_SQR2_SQ8				5
#define ADC_SQR2_SQ9				10
#define ADC_SQR2_SQ10				15
#define ADC_SQR2_SQ11				20
#define ADC_SQR2_SQ12				25
#define ADC_SQR2_Reserved			30

/*
 * Bit position definition for ADC_SQR3
 */

#define ADC_SQR3_SQ1				0
#define ADC_SQR3_SQ2				5
#define ADC_SQR3_SQ3				10
#define ADC_SQR3_SQ4				15
#define ADC_SQR3_SQ5				20
#define ADC_SQR3_SQ6				25
#define ADC_SQR3_Reserved			30

/*
 * Bit position definition for ADC_JSQR
 */

#define ADC_JSQR_JSQ1				0
#define ADC_JSQR_JSQ2				5
#define ADC_JSQR_JSQ3				10
#define ADC_JSQR_JSQ4				15
#define ADC_JSQR_JL					20
#define ADC_JSQR_Reserved			22

/*
 * Bit position definition for ADC_CSR
 */

#define ADC_CSR_ADC1_AWD1			0
#define ADC_CSR_ADC1_EOC1			1
#define ADC_CSR_ADC1_JEOC 1			2
#define ADC_CSR_ADC1_JSTRT1			3
#define ADC_CSR_ADC1_STRT1			4
#define ADC_CSR_ADC1_OVR1			5
#define ADC_CSR_ADC1_Reserved		6
#define ADC_CSR_ADC2_AWD2			8
#define ADC_CSR_ADC2_EOC2			9
#define ADC_CSR_ADC2_JEOC2			10
#define ADC_CSR_ADC2_JSTRT2			11
#define ADC_CSR_ADC2_STRT2			12
#define ADC_CSR_ADC2_OVR2			13
#define ADC_CSR_ADC2_Reserved2		14
#define ADC_CSR_ADC3_AWD3			16
#define ADC_CSR_ADC3_EOC3			17
#define ADC_CSR_ADC3_JEOC3			18
#define ADC_CSR_ADC3_JSTRT3			19
#define ADC_CSR_ADC3_STRT3			20
#define ADC_CSR_ADC3_OVR3			21
#define ADC_CSR_ADC3_Reserved3		22

/*
 * Bit position definition for ADC_CCR
 */

#define ADC_CCR_MULTI				0
#define ADC_CCR_Reserved1			5
#define ADC_CCR_DELAY				8
#define ADC_CCR_Reserved2			12
#define ADC_CCR_DDS					13
#define ADC_CCR_DMA					14
#define ADC_CCR_ADCPRE				16
#define ADC_CCR_Reserved3			18
#define ADC_CCR_VBATE				22
#define ADC_CCR_TSVREFE				23
#define ADC_CCR_Reserved4			24


/*======================================================================================
 *  Bit position definitions of RCC Registers
 *=======================================================================================*/

/*
 * Bit position definition for RCC_CR
 */

#define RCC_CR_HSION				0
#define RCC_CR_HSIRDY				1
#define RCC_CR_HSITRIM				3
#define RCC_CR_HSICAL				8
#define RCC_CR_HSEON				16
#define RCC_CR_HSERDY				17
#define RCC_CR_HSEBYP				18
#define RCC_CR_CSSON				19
#define RCC_CR_PLLON				24
#define RCC_CR_PLLRDY				25
#define RCC_CR_PLLI2SON				26
#define RCC_CR_PLLI2SRDY			27


/*
 * Bit position definition for RCC_CFGR
 */
#define RCC_CFGR_SW					0
#define RCC_CFGR_SWS				2
#define RCC_CFGR_HPRE				4
#define RCC_CFGR_PPRE1				10
#define RCC_CFGR_PPRE2				13
#define RCC_CFGR_RTCPRE				16
#define RCC_CFGR_MCO1				21
#define RCC_CFGR_I2SSCR				23
#define RCC_CFGR_MCO1PRE			24
#define RCC_CFGR_MCO2PRE			27
#define RCC_CFGR_MCO2				30


/*
 * Bit position definition for RCC_PLLCFGR
 */

#define RCC_PLLCFGR_PLLM			0
#define RCC_PLLCFGR_PLLN			6
#define RCC_PLLCFGR_PLLP			16
#define RCC_PLLCFGR_PLLSRC			22
#define RCC_PLLCFGR_PLLQ			24



/*
 * Bit position definition for RCC_CIR
 */


#define RCC_CIR_LSIRDYF				0
#define RCC_CIR_LSERDYF				1
#define RCC_CIR_HSIRDYF				2
#define RCC_CIR_HSERDYF				3
#define RCC_CIR_PLLRDYF				4
#define RCC_CIR_PLLI2SRDYF			5
#define RCC_CIR_CSSF				7
#define RCC_CIR_LSIRDYIE			8
#define RCC_CIR_LSERDYIE			9
#define RCC_CIR_HSIRDYIE			10
#define RCC_CIR_HSERDYIE			11
#define RCC_CIR_PLLRDYIE			12
#define RCC_CIR_PLLI2SRDYIE			13
#define RCC_CIR_LSIRDYC				16
#define RCC_CIR_LSERDYC				17
#define RCC_CIR_HSIRDYC				18
#define RCC_CIR_HSERDYC				19
#define RCC_CIR_PLLRDYC				20
#define RCC_CIR_PLLI2SRDYC			21
#define RCC_CIR_CSSC				23


/*
 * Bit position definition for RCC_AHB1RSTR
 */

#define RCC_AHB1RSTR_GPIOARST		0
#define RCC_AHB1RSTR_GPIOBRST		1
#define RCC_AHB1RSTR_GPIOCRST		2
#define RCC_AHB1RSTR_GPIODRST		3
#define RCC_AHB1RSTR_GPIOERST		4
#define RCC_AHB1RSTR_GPIOFRST		5
#define RCC_AHB1RSTR_GPIOGRST		6
#define RCC_AHB1RSTR_GPIOHRST		7
#define RCC_AHB1RSTR_GPIOIRST		8
#define RCC_AHB1RSTR_CRCRST			12
#define RCC_AHB1RSTR_DMA1RST		21
#define RCC_AHB1RSTR_DMA2RST		22
#define RCC_AHB1RSTR_ETHMACRST		25
#define RCC_AHB1RSTR_OTGHSRST		29


/*
 * Bit position definition for RCC_AHB2RSTR
 */

#define RCC_AHB2RSTR_DCMIRST		0
#define RCC_AHB2RSTR_CRYPRST		4
#define RCC_AHB2RSTR_HASHRST		5
#define RCC_AHB2RSTR_RNGRST			6
#define RCC_AHB2RSTR_OTGFSRST		7


/*
 * Bit position definition for RCC_AHB3RSTR
 */

#define RCC_AHB3RSTR_FSMCRST		0


/*
 * Bit position definition for RCC_APB1RSTR
 */

#define RCC_APB1RSTR_TIM2RST		0
#define RCC_APB1RSTR_TIM3RST		1
#define RCC_APB1RSTR_TIM4RST		2
#define RCC_APB1RSTR_TIM5RST		3
#define RCC_APB1RSTR_TIM6RST		4
#define RCC_APB1RSTR_TIM7RST		5
#define RCC_APB1RSTR_TIM12RST		6
#define RCC_APB1RSTR_TIM13RST		7
#define RCC_APB1RSTR_TIM14RST		8
#define RCC_APB1RSTR_WWDGRST		11
#define RCC_APB1RSTR_SPI2RST		14
#define RCC_APB1RSTR_SPI3RST		15
#define RCC_APB1RSTR_UART2RST		17
#define RCC_APB1RSTR_UART3RST		18
#define RCC_APB1RSTR_UART4RST		19
#define RCC_APB1RSTR_UART5RST		20
#define RCC_APB1RSTR_I2C1RST		21
#define RCC_APB1RSTR_I2C2RST		22
#define RCC_APB1RSTR_I2C3RST		23
#define RCC_APB1RSTR_CAN1RST		25
#define RCC_APB1RSTR_CAN2RST		26
#define RCC_APB1RSTR_PWRRST			28
#define RCC_APB1RSTR_DACRST			29


/*
 * Bit position definition for RCC_APB2RSTR
 */

#define RCC_APB2RSTR_TIM1RST		0
#define RCC_APB2RSTR_TIM8RST		1
#define RCC_APB2RSTR_USART1RST		4
#define RCC_APB2RSTR_USART6RST		5
#define RCC_APB2RSTR_ADCRST			8
#define RCC_APB2RSTR_SDIORST		11
#define RCC_APB2RSTR_SPI1RST		12
#define RCC_APB2RSTR_SYSCFGRST		14
#define RCC_APB2RSTR_TIM9RST		16
#define RCC_APB2RSTR_TIM10RST		17
#define RCC_APB2RSTR_TIM11RST		18


/*
 * Bit position definition for RCC_AHB1ENR
 */

#define RCC_AHB1ENR_GPIOAEN			0
#define RCC_AHB1ENR_GPIOBEN			1
#define RCC_AHB1ENR_GPIOCEN			2
#define RCC_AHB1ENR_GPIODEN			3
#define RCC_AHB1ENR_GPIOEEN			4
#define RCC_AHB1ENR_GPIOFEN			5
#define RCC_AHB1ENR_GPIOGEN			6
#define RCC_AHB1ENR_GPIOHEN			7
#define RCC_AHB1ENR_GPIOIEN			8
#define RCC_AHB1ENR_CRCEN			12
#define RCC_AHB1ENR_BKPSRAMEN		18
#define RCC_AHB1ENR_CCMDATARAMEN	20
#define RCC_AHB1ENR_DMA1EN			21
#define RCC_AHB1ENR_DMA2EN			22
#define RCC_AHB1ENR_ETHMACEN		25
#define RCC_AHB1ENR_ETHMACTXEN		26
#define RCC_AHB1ENR_ETHMACRXEN		27
#define RCC_AHB1ENR_ETHMACPTPEN		28
#define RCC_AHB1ENR_OTGHSEN			29
#define RCC_AHB1ENR_OTGHSULPIEN		30


/*
 * Bit position definition for RCC_AHB2ENR
 */

#define RCC_AHB2ENR_DCMIEN			0
#define RCC_AHB2ENR_CRYPEN			4
#define RCC_AHB2ENR_HASHEN			5
#define RCC_AHB2ENR_RNGEN			6
#define RCC_AHB2ENR_OTGFSEN			7


/*
 * Bit position definition for RCC_AHB3ENR
 */

#define RCC_AHB3ENR_FSMCEN			0


/*
 * Bit position definition for RCC_APB1ENR
 */

#define RCC_APB1ENR_TIM2EN			0
#define RCC_APB1ENR_TIM3EN			1
#define RCC_APB1ENR_TIM4EN			2
#define RCC_APB1ENR_TIM5EN			3
#define RCC_APB1ENR_TIM6EN			4
#define RCC_APB1ENR_TIM7EN			5
#define RCC_APB1ENR_TIM12EN			6
#define RCC_APB1ENR_TIM13EN			7
#define RCC_APB1ENR_TIM14EN			8
#define RCC_APB1ENR_WWDGEN			11
#define RCC_APB1ENR_SPI2EN			14
#define RCC_APB1ENR_SPI3EN			15
#define RCC_APB1ENR_USART2EN		17
#define RCC_APB1ENR_USART3EN		18
#define RCC_APB1ENR_UART4EN			19
#define RCC_APB1ENR_UART5EN			20
#define RCC_APB1ENR_I2C1EN			21
#define RCC_APB1ENR_I2C2EN			22
#define RCC_APB1ENR_I2C3EN			23
#define RCC_APB1ENR_CAN1EN			25
#define RCC_APB1ENR_CAN2EN			26
#define RCC_APB1ENR_PWREN			28
#define RCC_APB1ENR_DACEN			29


/*
 * Bit position definition for RCC_APB2ENR
 */

#define RCC_APB2ENR_TIM1EN			0
#define RCC_APB2ENR_TIM8EN			1
#define RCC_APB2ENR_USART1EN		4
#define RCC_APB2ENR_USART6EN		5
#define RCC_APB2ENR_ADC1EN			8
#define RCC_APB2ENR_ADC2EN			9
#define RCC_APB2ENR_ADC3EN			10
#define RCC_APB2ENR_SDIOEN			11
#define RCC_APB2ENR_SPI1EN			12
#define RCC_APB2ENR_SYSCFGEN		14
#define RCC_APB2ENR_TIM9EN			16
#define RCC_APB2ENR_TIM10EN			17
#define RCC_APB2ENR_TIM11EN			18


/*
 * Bit position definition for RCC_AHB1LPENR
 */

#define RCC_AHB1LPENR_GPIOALPEN			0
#define RCC_AHB1LPENR_GPIOBLPEN			1
#define RCC_AHB1LPENR_GPIOCLPEN			2
#define RCC_AHB1LPENR_GPIODLPEN			3
#define RCC_AHB1LPENR_GPIOELPEN			4
#define RCC_AHB1LPENR_GPIOFLPEN			5
#define RCC_AHB1LPENR_GPIOGLPEN			6
#define RCC_AHB1LPENR_GPIOHLPEN			7
#define RCC_AHB1LPENR_GPIOILPEN			8
#define RCC_AHB1LPENR_CRCLPEN			12
#define RCC_AHB1LPENR_FLITFLPEN			15
#define RCC_AHB1LPENR_SRAM1LPEN			16
#define RCC_AHB1LPENR_SRAM2LPEN			17
#define RCC_AHB1LPENR_BKPSRAMLPEN		18
#define RCC_AHB1LPENR_DMA1LPEN			21
#define RCC_AHB1LPENR_DMA2LPEN			22
#define RCC_AHB1LPENR_ETHMACLPEN		25
#define RCC_AHB1LPENR_ETHMACTXLPEN		26
#define RCC_AHB1LPENR_ETHMACRXLPEN		27
#define RCC_AHB1LPENR_ETHMACPTPLPEN		28
#define RCC_AHB1LPENR_OTGHSLPEN			29
#define RCC_AHB1LPENR_OTGHSULPILPEN		30

/*
 * Bit position definition for RCC_AHB2LPENR
 */

#define RCC_AHB2LPENR_DCMILPEN		0
#define RCC_AHB2LPENR_CRYPLPEN		4
#define RCC_AHB2LPENR_HASHLPEN		5
#define RCC_AHB2LPENR_RNGLPEN		6
#define RCC_AHB2LPENR_OTGFSLPEN		7


/*
 * Bit position definition for RCC_AHB3LPENR
 */

#define RCC_AHB3LPENR_FSMCLPEN		0

//===================== START FROM HERE
/*
 * Bit position definition for RCC_APB1LPENR
 */

#define RCC_AHB3LPENR_FSMCLPEN		0

/*
 * Bit position definition for RCC_BDCR
 */


#define RCC_BDCR_LSEON				0
#define RCC_BDCR_LSERDY				1
#define RCC_BDCR_LSEBYP				2
#define RCC_BDCR_RTCSEL				8
#define RCC_BDCR_RTCEN				15
#define RCC_BDCR_BDRST				16

/*
 * Bit position definition for RCC_CSR
 */


#define RCC_CSR_LSION				0
#define RCC_CSR_LSIRDY				1
#define RCC_CSR_RMVF				24
#define RCC_CSR_BORRSTF				25
#define RCC_CSR_PINRSTF				26
#define RCC_CSR_PORRSTF				27
#define RCC_CSR_SFTRSTF				28
#define RCC_CSR_IWDGRSTF			29
#define RCC_CSR_WWDGRSTF			30
#define RCC_CSR_LPWRRSTF			30



/*======================================================================================
 *  Bit position definitions of TIMER peripherals
 *=======================================================================================*/

/*TIMx control register 1
 * Bit position definition for TIMx_CR1
 */

#define TIMx_CR1_CEN				0		/*<Bit is used for configuring Tdts >*/
#define TIMx_CR1_UDIS				1		/*<Bit is used for Auto-reload preload enable  >*/
#define TIMx_CR1_URS				2		/*<Bit is used for Center-aligned mode selection >*/
#define TIMx_CR1_OPM				3		/*<possible value from @GPIO_PIN_NUMBERS >*/
#define TIMx_CR1_DIR				4
#define TIMx_CR1_CMS				5
#define TIMx_CR1_ARPE				7
#define TIMx_CR1_CKD				8

/*
 * Bit position definition for TIMx_CR2
 */

#define TIMx_CR2_CCDS				3
#define TIMx_CR2_MMS				4
#define TIMx_CR2_TI1S				7

/*
 * Bit position definition for TIMx_SMCR
 */

#define TIMx_SMCR_SMS				0
#define TIMx_SMCR_TS				4
#define TIMx_SMCR_MSM				7
#define TIMx_SMCR_ETF				8
#define TIMx_SMCR_ETPS				12
#define TIMx_SMCR_ECE				14
#define TIMx_SMCR_ETP				15

/*
 * Bit position definition for TIMx_DIER
 */

#define TIMx_DIER_UIE				0
#define TIMx_DIER_CC1IE				1
#define TIMx_DIER_CC2IE				2
#define TIMx_DIER_CC3IE				3
#define TIMx_DIER_CC4IE				4
#define TIMx_DIER_TIE				6
#define TIMx_DIER_UDE				8
#define TIMx_DIER_CC1DE				9
#define TIMx_DIER_CC2DE				10
#define TIMx_DIER_CC3DE				11
#define TIMx_DIER_CC4DE				12
#define TIMx_DIER_TDE				14



/*
 * Bit position definition for TIMx_SR
 */

#define TIMx_SR_UIF					0
#define TIMx_SR_CC1IF				1
#define TIMx_SR_CC2IF				2
#define TIMx_SR_CC3IF				3
#define TIMx_SR_CC4IF				4
#define TIMx_SR_TIF					6
#define TIMx_SR_CC1OF				9
#define TIMx_SR_CC2OF				10
#define TIMx_SR_CC3OF				11
#define TIMx_SR_CC4OF				12


/*
 * Bit position definition for TIMx_EGR
 */

#define TIMx_EGR_UG					0
#define TIMx_EGR_CC1G				1
#define TIMx_EGR_CC2G				2
#define TIMx_EGR_CC3G				3
#define TIMx_EGR_CC4G				4
#define TIMx_EGR_TG					6

/*TIMx capture/compare mode register
 *
 * Bit position definition for TIMx_CCMR1
 */

#define TIMx_CCMR1_CC1S					0
#define TIMx_CCMR1_IC1PSC				2
#define TIMx_CCMR1_IC1F					4
#define TIMx_CCMR1_OC1FE				2
#define TIMx_CCMR1_OC1PE				3
#define TIMx_CCMR1_OC1M					4
#define TIMx_CCMR1_OC1CE				7

#define TIMx_CCMR1_CC2S					8
#define TIMx_CCMR1_IC2PSC				10
#define TIMx_CCMR1_IC2F					12
#define TIMx_CCMR1_OC2FE				10
#define TIMx_CCMR1_OC2PE				11
#define TIMx_CCMR1_OC2M					12
#define TIMx_CCMR1_OC2CE				15

/*TIMx capture/compare mode register
 *
 * Bit position definition for TIMx_CCMR2
 */

#define TIMx_CCMR2_CC3S					0
#define TIMx_CCMR2_IC3PSC				2
#define TIMx_CCMR2_IC3F					4
#define TIMx_CCMR2_OC3FE				2
#define TIMx_CCMR2_OC3PE				3
#define TIMx_CCMR2_OC3M					4
#define TIMx_CCMR2_OC3CE				7

#define TIMx_CCMR2_CC4S					8
#define TIMx_CCMR2_IC4PSC				10
#define TIMx_CCMR2_IC4F					12
#define TIMx_CCMR2_OC4FE				10
#define TIMx_CCMR2_OC4PE				11
#define TIMx_CCMR2_OC4M					12
#define TIMx_CCMR2_OC4CE				15

/*TIMx capture/compare enable register
 *
 * Bit position definition for TIMx_CCER
 */

#define TIMx_CCER_CC1E					0
#define TIMx_CCER_CC1P					1
#define TIMx_CCER_CC1NP					3
#define TIMx_CCER_CC2E					4
#define TIMx_CCER_CC2P					5
#define TIMx_CCER_CC2NP					7
#define TIMx_CCER_CC3E					8
#define TIMx_CCER_CC3P					9
#define TIMx_CCER_CC3NP					11
#define TIMx_CCER_CC4E					12
#define TIMx_CCER_CC4P					13
#define TIMx_CCER_CC4NP					15

/*TIMx DMA control register
 *
 * Bit position definition for TIMx_DCR
 */

#define TIMx_DCR_DBA					0
#define TIMx_DCR_DBL					0







#endif /* INC_STM32F407XX_H_ */
