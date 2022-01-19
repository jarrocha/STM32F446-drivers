/*
 * 002_LED_ButtonToggle.c
 *
 *  Created on: Oct 28, 2021
 *      Author: jaime
 */

#include <stdint.h>
#include "stm32f446xx_gpio_driver.h"

void delay()
{
	for (uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
	// Configure Button
	GPIO_Handle_t config;
	config.pGPIOx = GPIOC;
	config.pGPIOx_pin.GPIO_PinNumber = 13;
	config.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_Init(&config);

	// Configure LED
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.pGPIOx_pin.GPIO_PinNumber = 5;
	GpioLed.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.pGPIOx_pin.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;

	GPIO_Init(&GpioLed);

	for(;;) {
		if (GPIO_ReadFromInputPin(GPIOC, 13) == 0) {

			// To remove bit banging
			delay();
			GPIO_ToggleOutputPin(GPIOA, 5);
		}
	}

	return 0;
}
