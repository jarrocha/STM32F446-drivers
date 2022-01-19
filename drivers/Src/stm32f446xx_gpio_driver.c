/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Jaime
 */
#include <stdio.h>
#include "stm32f446xx_gpio_driver.h"

// Prototypes
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enableState);

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	if (pGPIOHandle == NULL) {
		printf("LOG: error: invalid handle.\n");
		return;
	}

	// Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint8_t pinNum = pGPIOHandle->pGPIOx_pin.GPIO_PinNumber;
	uint8_t pinMode = pGPIOHandle->pGPIOx_pin.GPIO_PinMode;
	GPIO_RegDef_t* pGPIORef = pGPIOHandle->pGPIOx;

	// Configure GPIO mode
	if (pinMode <= GPIO_MODE_ANALOG) {
		// The MODE register uses its 32-bits capacity to address all 15 pins. 
		uint32_t bits = (pinMode << (2 * pinNum));

		// clearing the bits before setting them
		pGPIORef->MODER &= ~(bits);
		pGPIORef->MODER |= (bits);

		// Configure GPIO alternate function
		if (pinMode == GPIO_MODE_ALTFN) {
			if (pinNum >= 8) {
				pGPIORef->AFRH |= (pGPIOHandle->pGPIOx_pin.GPIO_PinAltFunMode << (4 * (pinNum - 8)));
			} else {
				pGPIORef->AFRL |= (pGPIOHandle->pGPIOx_pin.GPIO_PinAltFunMode << (4 * pinNum));
			}
		}

	} else {
		// TODO: use switch for better visibility. Improve comments.

		// Falling trigger, configure FTSR
		if (pinMode == GPIO_MODE_IT_FT) {
			// Configure FTSR
			EXTI->FTSR |= (1 << pinNum);

			// Clear the corresponding RTSR bit
			// This is done because these methods can be done together
			EXTI->RTSR &= ~(1 << pinNum);
		} else if (pinMode == GPIO_MODE_IT_RT) {
			// Falling trigger, configure RTSR
			// Configure FTSR
			EXTI->RTSR |= (1 << pinNum);

			// Clear the corresponding RTSR bit
			// This is done because these methods can be done together
			EXTI->FTSR &= ~(1 << pinNum);
		} else if (pinMode == GPIO_MODE_IT_RFT) {
			// Configure both registers FTSR & RTSR

			// Configure FTSR
			EXTI->FTSR |= (1 << pinNum);
			EXTI->RTSR |= (1 << pinNum);
		}

		// Setup interrupt functionality
		GPIO_IRQInit(pGPIORef, pinNum);
	}

	// Configure GPIO speed
	pGPIORef->OSPEEDR = (pGPIOHandle->pGPIOx_pin.GPIO_PinSpeed << (2 * pinNum));

	// Configure GPIO pupd settings
	pGPIORef->PUPDR = (pGPIOHandle->pGPIOx_pin.GPIO_PinPuPdControl << (2 * pinNum));

	// Configure GPIO output type
	pGPIORef->OTYPER = (pGPIOHandle->pGPIOx_pin.GPIO_PinOPType << (pinNum));

}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	GPIO_PeriClockControl(pGPIOx, DISABLE);
}

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enableState)
{
	if (pGPIOx == NULL) {
		printf("LOG: error: invalid port.\n");
		return;
	}

	if (enableState) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else {
			printf("LOG: error: invalid port.\n");
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else {
			printf("LOG: error: invalid port.\n");
		}
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	// the mask 0x00000001 is used to clean the output
	return (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR);
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value)
{
	uint8_t val = 1 << pinNumber;
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= val;
	} else {
		pGPIOx->ODR &= ~(val);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint8_t value)
{
	pGPIOx->ODR = value;	
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNnumber, uint8_t enableState)
{
	if (enableState == ENABLE) {
		if (IRQNnumber <= 31) { 
			// program NVIC_ISER0 register
			*NVIC_ISER0 |= (1 << IRQNnumber);
		} else if (IRQNnumber > 31 && IRQNnumber < 64) {
			// program NVIC_ISER1 register
			*NVIC_ISER1 |= (1 << IRQNnumber % 32);
		} else if (IRQNnumber >= 64 && IRQNnumber < 96) {
			// program NVIC_ISER2 register
			*NVIC_ISER2 |= (1 << IRQNnumber % 64);
		}
	} else {
		if (IRQNnumber <= 31) { 
			// program NVIC_ICER0 register
			*NVIC_ICER0 |= (1 << IRQNnumber);
		} else if (IRQNnumber > 31 && IRQNnumber < 64) {
			// program NVIC_ICER1 register
			*NVIC_ICER1 |= (1 << IRQNnumber % 32);
		} else if (IRQNnumber >= 64 && IRQNnumber < 96) {
			// program NVIC_ICER2 register
			*NVIC_ICER2 |= (1 << IRQNnumber % 64);
		}
	}
}

void GPIO_IRQHandle(uint8_t pinNumber)
{
	// clear the EXTI peripheral register corresponding to the pin number
	if (EXTI->PR & (1 << pinNumber)) {
		// clear
		EXTI->PR |= (1 << pinNumber);
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNnumber, uint32_t IRQPriority)
{
	// This obtains the offset for the 60 priority register, multiplied by their size.
	uint32_t nvic_ipr_reg_idx = (IRQNnumber / 4);

	// This is to obtain which section within the register to modify.
	uint8_t ipr_reg_section_idx = ((IRQNnumber % 4) * 8);

	// We only use the upper half of the register for values.
	ipr_reg_section_idx += (8 - NO_PR_BITS_IMPLEMENTED);

	// NOTE: integer addition below
	*(NVIC_PR_BADDR+nvic_ipr_reg_idx) |= (IRQPriority << ipr_reg_section_idx);
}

void GPIO_IRQInit(GPIO_RegDef_t* gp, uint8_t pinNumber)
{
	// Enable SYSCFG peripheral clock
	SYSCFG_PCLK_EN();

	/*
		Explanation for procedure below:
		The microcontroller STM32F446XX has 16 external interrupt/event lines which can be used to enable
		interrupts for all 114 GPIOs through the use of what is called EXTI registers.

		To accomplish this, we use 4 EXTI 32-bit control registers and within them, 
		4 EXTI indexes 4-bit in length to help select from 8 different GPIO ports.

		Also, depending on the EXTI port using, we enable the processor NVIC line to use.
	*/

	// First, select the SYSCFG_EXTI register
	uint8_t regIdx = pinNumber / 4;					// Identify SYSCFG_EXTI control register index
	uint8_t extiIdx = (pinNumber % 4) * 4;			// Identify EXTI register offset
	
	// Once the index and offset are computed, we need to get the value to store. The value should be the
	// port number in use (0-7), offset by the number calculated above.
	uint8_t portNum = GET_GPIO_NUM_FROM_PTR(gp);

	// Entering the values calculated
	SYSCFG->EXTICR[regIdx] = (portNum << extiIdx);

	// Enable EXTI interrupt delivery using IMR (Interrupt Mask Register)
	EXTI->IMR |= 1 << pinNumber;					// Disabling mask to given pin number
}
