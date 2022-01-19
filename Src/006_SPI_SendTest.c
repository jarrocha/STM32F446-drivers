/*
 * 006_SPI_SendTest.c
 *
 *  Created on: Dec 8, 2021
 *      Author: jaime
 */

/** Setup
 * SPI2_NSS  --> PB12 AF5
 * SPI2_SCLK --> PB13 AF5
 * SPI2_MISO --> PB14 AF5
 * SPI2_MOSI --> PB15 AF5
 */
#include <stdint.h>
#include <string.h>
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

void PinInit()
{
	// Configure MOSI Pin
	GPIO_Handle_t spiPin;
	memset(&spiPin, 0, sizeof(spiPin));

	// Both pins share similar configurations
	spiPin.pGPIOx = GPIOB;
	spiPin.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_ALTFN;
	spiPin.pGPIOx_pin.GPIO_PinAltFunMode = BIT5;
	spiPin.pGPIOx_pin.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	spiPin.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spiPin.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	// Setting up MOSI pin
	spiPin.pGPIOx_pin.GPIO_PinNumber = BIT15;
	GPIO_Init(&spiPin);

	// Setting up SCLK pin
	spiPin.pGPIOx_pin.GPIO_PinNumber = BIT13;
	GPIO_Init(&spiPin);
}

int main()
{
	PinInit();

	// Configure spi2 for master communication
	SPI_Handle_t spi2;
	memset(&spi2, 0, sizeof(spi2));

	spi2.pSPIx = SPI2;
	spi2.SPIConfig.SPI_BusConfig = SPI_BUS_CFG_FULLD;
	spi2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPIConfig.SPI_Speed = SPI_CLK_SPEED_DIV2;
	spi2.SPIConfig.SPI_DFF = SPI_DFF_MODE_8BIT;
	spi2.SPIConfig.SPI_CPHA = SPI_CLKPHA_LOW;
	spi2.SPIConfig.SPI_CPOL = SPI_CLKPOL_LOW;
	spi2.SPIConfig.SPI_SSM = SPI_SSM_ENABLE;
	SPI_Init(&spi2);

	// Manually setting SSI to high to avoid modf error.
	SPI_SSIConfig(spi2.pSPIx, ENABLE);

	// Enable device after configuration
    SPI_SetupDevice(spi2.pSPIx, ENABLE);

	// Send data
	char buff[] = "Hello World";
	SPI_SendData(spi2.pSPIx, (uint8_t*)buff, strlen(buff));

	while (1)
	{
		/* code */
	}

	return 0;
}
