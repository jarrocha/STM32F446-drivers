/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Oct 21, 2021
 *      Author: jaime
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/**
 * GPIO in posible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// Falling edge detection. Requires IRQ mode.
#define GPIO_MODE_IT_RT		5		// Rising edge detection. Requires IRQ mode.
#define GPIO_MODE_IT_RFT	6		// Rising-Falling edge detection. Requires IRQ mode.

/**
 * GPIO in posible output type
 */
#define GPIO_OUTPUT_TYPE_PP	0		// Output push-pull
#define GPIO_OUTPUT_TYPE_OD	1		// Output open-drain

/**
 * GPIO in posible speed modes
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/**
 * GPIO pull-up/up-down modes
 */
#define GPIO_PUPD_NONE		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2

// Configuration structure for GPIO pin
typedef struct {
	uint8_t	GPIO_PinNumber;				// GPIO bit pin number
	uint8_t	GPIO_PinMode;				// GPIO mode selection: GPIO_MODE_XX 
	uint8_t	GPIO_PinSpeed;				// GPIO speed selection: GPIO_SPEED_XX 
	uint8_t	GPIO_PinPuPdControl;		// GPIO pull up/down control: GPIO_PUPD_XX 
	uint8_t	GPIO_PinOPType;				// GPIO output mode: GPIO_OUTPUT_TYPE_XX
	uint8_t	GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t* 		pGPIOx;			// Hold corresponding GPIO port
	GPIO_PinConfig_t	pGPIOx_pin;		// Hold configuration
} GPIO_Handle_t;

/**
 * 	 Init and reset GPIO port
 */
// Initializes GPIO configuration parameters and enables peripheral
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/**
 * 	 Read and Write to pin and port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);

/**
 * 	 IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNnumber, uint8_t enableState);
void GPIO_IRQPriorityConfig(uint8_t IRQNnumber, uint32_t IRQPriority);
void GPIO_IRQHandle(uint8_t pinNumber);
void GPIO_IRQInit(GPIO_RegDef_t* gp, uint8_t pinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
