/*
 * 002_LED_ButtonToggle.c
 *
 *  Created on: Oct 28, 2021
 *      Author: jaime
 */

#include <stdint.h>
#include <string.h>
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

void delay()
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void EXTI15_10_IRQHandler(void)
{
	delay(); // wait 200 ms

	// Pin number for peripheral starting the IRQ.
	GPIO_IRQHandle(GPIO_PIN_13);

	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
}

int main(void)
{
	// Configure Button
	GPIO_Handle_t Button;
	memset(&Button, 0, sizeof(Button));

	Button.pGPIOx = GPIOC;
	Button.pGPIOx_pin.GPIO_PinNumber = GPIO_PIN_13;
	Button.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_PeriClockControl(Button.pGPIOx, ENABLE);
	GPIO_Init(&Button);

	// Configure LED
	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0, sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.pGPIOx_pin.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.pGPIOx_pin.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	// IRQ setup
	GPIO_IRQPriorityConfig(IRQ_NO_EXT15_10, 15);

	// For config, we need the EXTI number used by button at PC13
	GPIO_IRQConfig(IRQ_NO_EXT15_10, ENABLE);

	for(;;) {}

	return 0;
}
