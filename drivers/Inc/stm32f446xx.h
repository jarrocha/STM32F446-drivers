/*
 * stm32f446xx.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Jaime
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*******************************************************
 *	Processor Specific Details - START
 *  Memory map for region 0XE000 0000 - 0xE00F FFFF used
 *  for Cortex-M4 internal peripherals.
 *******************************************************/

/**
 *	Interrupt set-enable registers  
 */
#define NVIC_ISER0			((__vo uint32_t*) 0xE000E100U)
#define NVIC_ISER1			((__vo uint32_t*) 0xE000E104U)
#define NVIC_ISER2			((__vo uint32_t*) 0xE000E108U)
#define NVIC_ISER3			((__vo uint32_t*) 0xE000E10CU)

/**
 *	Interrupt clear-enable registers
 */
#define NVIC_ICER0			((__vo uint32_t*) 0xE000E180U)
#define NVIC_ICER1			((__vo uint32_t*) 0xE000E184U)
#define NVIC_ICER2			((__vo uint32_t*) 0xE000E188U)
#define NVIC_ICER3			((__vo uint32_t*) 0xE000E18CU)

/**
 *	Interrupt Priority registers
 */
#define NVIC_PR_BADDR		((__vo uint32_t*) 0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4



/*******************************************************
 *	Board Specific Details
 *******************************************************/
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

/**
 * EXTI IRQ Position Number for NVIC
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI5_9			23
#define IRQ_NO_EXT15_10			40

/**
 * Main memory addresses
 */
#define FLASH_BADDR			0x08000000U			//  512 KB
#define SRAM1_BADDR			0x20000000U			//	112 KB
#define SRAM2_BADDR			0x2001C000U			//	 16 KB
#define ROM_BADDR			0x1FFF0000U			// 	 30 KB
#define SRAM_BADDR			SRAM1_BASE_ADDR

/**
 * Bus addresses
 */
#define P_BADDR				0x40000000U			// Base peripheral address
#define APB1P_BADDR			P_BADDR
#define APB2P_BADDR			0x40010000U
#define AHB1P_BADDR			0x40020000
#define AHB2P_BADDR			0x50000000
#define AHB3P_BADDR			0x60000000

/**
 * AHB1 Peripherals
 */
#define RCC_BADDR			(AHB1P_BADDR + 0x3800)
#define GPIOA_BADDR			(AHB1P_BADDR + 0x0000)
#define GPIOB_BADDR			(AHB1P_BADDR + 0x0400)
#define GPIOC_BADDR			(AHB1P_BADDR + 0x0800)
#define GPIOD_BADDR			(AHB1P_BADDR + 0x0C00)
#define GPIOE_BADDR			(AHB1P_BADDR + 0x1000)
#define GPIOF_BADDR			(AHB1P_BADDR + 0x1400)
#define GPIOG_BADDR			(AHB1P_BADDR + 0x1800)
#define GPIOH_BADDR			(AHB1P_BADDR + 0x1C00)

/**
 * APB1 Peripherals
 */
#define I2C1_BADDR			(APB1P_BADDR + 0x5400)
#define I2C2_BADDR			(APB1P_BADDR + 0x5800)
#define I2C3_BADDR			(APB1P_BADDR + 0x5C00)
#define SPI2_BADDR			(APB1P_BADDR + 0x3800)
#define SPI3_BADDR			(APB1P_BADDR + 0x3C00)
#define USART2_BADDR		(APB1P_BADDR + 0x4400)
#define USART3_BADDR		(APB1P_BADDR + 0x4800)
#define UART4_BADDR			(APB1P_BADDR + 0x4C00)
#define UART5_BADDR			(APB1P_BADDR + 0x5000)

/**
 * APB2 Peripherals
 */
#define SPI1_BADDR			(APB2P_BADDR + 0x3000)
#define SPI4_BADDR			(APB2P_BADDR + 0x3400)
#define SYSCFG_BADDR		(APB2P_BADDR + 0x3800)
#define EXTI_BADDR			(APB2P_BADDR + 0x3C00)

/**
 * RCC register definition
 */
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1_RSTR;
	__vo uint32_t AHB2_RSTR;
	__vo uint32_t AHB3_RSTR;
	uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/**
 * RCC definition
 */
#define RCC		((RCC_RegDef_t*)RCC_BADDR)

/**
 * GPIO Peripheral register definition
 */
typedef struct {
	__vo uint32_t MODER;				// GPIO port mode register
	__vo uint32_t OTYPER;				// GPIO port output type register
	__vo uint32_t OSPEEDR;				// GPIO port output speed register
	__vo uint32_t PUPDR;				// GPIO port pull-up/down register
	__vo uint32_t IDR;					// GPIO port input data register
	__vo uint32_t ODR;					// GPIO port output data register
	__vo uint32_t BSRR;					// GPIO port bit set/reset register
	__vo uint32_t LCKR;					// GPIO port configuration lock register
	__vo uint32_t AFRL;					// GPIO alternate function low register
	__vo uint32_t AFRH;					// GPIO alternate function high register
} GPIO_RegDef_t;

/**
 * GPIO definitions
 */
#define GPIOA 				((GPIO_RegDef_t*) GPIOA_BADDR)
#define GPIOB 				((GPIO_RegDef_t*) GPIOB_BADDR)
#define GPIOC 				((GPIO_RegDef_t*) GPIOC_BADDR)
#define GPIOD 				((GPIO_RegDef_t*) GPIOD_BADDR)
#define GPIOE 				((GPIO_RegDef_t*) GPIOE_BADDR)
#define GPIOF 				((GPIO_RegDef_t*) GPIOF_BADDR)
#define GPIOG 				((GPIO_RegDef_t*) GPIOG_BADDR)
#define GPIOH 				((GPIO_RegDef_t*) GPIOH_BADDR)

#define GET_GPIO_NUM_FROM_PTR(x)  	(((GPIO_RegDef_t*)x == GPIOA)?0:\
									((GPIO_RegDef_t*)x == GPIOB)?1:\
									((GPIO_RegDef_t*)x == GPIOC)?2:\
									((GPIO_RegDef_t*)x == GPIOD)?3:\
									((GPIO_RegDef_t*)x == GPIOE)?4:\
									((GPIO_RegDef_t*)x == GPIOF)?5:\
									((GPIO_RegDef_t*)x == GPIOG)?6:\
									((GPIO_RegDef_t*)x == GPIOH)?7:0)
/**
 * GPIO Enable Macros
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/**
 * I2C Enable Macros
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/**
 * SPI Enable Macros
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/**
 * UART Enable Macros
 */
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))

/**
 * SYSCFG Enable Macros
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/**
 * GPIO Disable Macros
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/**
 * GPIO Reset Macros
 */
#define GPIOA_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 0)); (RCC->AHB1_RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 1)); (RCC->AHB1_RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 2)); (RCC->AHB1_RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 3)); (RCC->AHB1_RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 4)); (RCC->AHB1_RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 5)); (RCC->AHB1_RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 6)); (RCC->AHB1_RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_RESET() 		do{ (RCC->AHB1_RSTR |= (1 << 7)); (RCC->AHB1_RSTR &= ~(1 << 7)); } while(0)

/**
 * I2C Disable Macros
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/**
 * SPI Disable Macros
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/**
 * UART Disable Macros
 */
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))

/**
 * SYSCFG Disable Macros
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/**
 *	EXTI Peripheral register definition 
 */
typedef struct {
	__vo uint32_t IMR;					// Interrupt mask register
	__vo uint32_t EMR;					// Event mask register
	__vo uint32_t RTSR;					// Rising trigger selection register
	__vo uint32_t FTSR;					// Falling trigger selection register
	__vo uint32_t SWIER;				// Software interrupt event register
	__vo uint32_t PR;					// Pending register
} EXTI_RegDef_t;

/**
 * EXTI definition
 */
#define EXTI ((EXTI_RegDef_t*) EXTI_BADDR)

/**
 *	SYSCFG Peripheral register definition 
 */
typedef struct {
	__vo uint32_t MEMRMP;				// SYSCFG memory remap register
	__vo uint32_t PMC;					// SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];			// SYSCFG external interrupt configuration registers
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;				// Compensation cell control register
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;					// SYSCFG configuration register
} SYSCFG_RegDef_t;

/**
 * SYSCFG definition
 */
#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BADDR)

/**
 *	SPI register definitiion
 */
typedef struct {
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
} SPI_RegDef_t;

#define SPI1	((SPI_RegDef_t*)SPI1_BADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BADDR)

/*******************************************************
 *	Common Definitions
 *******************************************************/
#define BIT0 	0
#define BIT1 	1
#define BIT2 	2
#define BIT3 	3
#define BIT4 	4
#define BIT5 	5
#define BIT6 	6
#define BIT7 	7
#define BIT8 	8
#define BIT9 	9
#define BIT10	10
#define BIT11	11
#define BIT12	12
#define BIT13	13
#define BIT14	14
#define BIT15	15
#define BIT16	16
#define BIT17	17
#define BIT18	18
#define BIT19	19
#define BIT20	20

/*******************************************************
 *	Utility Functions
 *******************************************************/
uint32_t TestBit(uint32_t register, uint32_t pos);

void Delay(uint32_t waitNum);

#endif /* INC_STM32F446XX_H_ */
