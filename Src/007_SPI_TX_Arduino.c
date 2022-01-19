/*
 * 007_SPI_TX_Arduino.c
 *
 *  Created on: Dec 8, 2019
 *      Author: Jaime A.
 */

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

/* SPI Slave Demo
 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *
 */

#include <stdint.h>
#include <string.h>
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

void SPI_GPIOinit()
{
	// Configuring GPIO for SCLK, MOSI and NSS.
//	GPIO_Handle_t spiPin;
////	memset(&spiPin, 0, sizeof(spiPin));
//
//	spiPin.pGPIOx = GPIOB;
//	spiPin.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_ALTFN;
//	spiPin.pGPIOx_pin.GPIO_PinAltFunMode = BIT5;
//	spiPin.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	spiPin.pGPIOx_pin.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
//	spiPin.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
//
//	// Configuring SCLK
//	spiPin.pGPIOx_pin.GPIO_PinNumber = BIT13;
//	GPIO_Init(&spiPin);
//
//	// Configuring MOSI
//	spiPin.pGPIOx_pin.GPIO_PinNumber = BIT15;
//	GPIO_Init(&spiPin);
//
//	// // Configuring NSS
//	// spiPin.pGPIOx_pin.GPIO_PinNumber = BIT12;
//	// GPIO_Init(&spiPin);



	// Configure MOSI Pin
 	GPIO_Handle_t mosiPin;
 	memset(&mosiPin, 0, sizeof(mosiPin));

 	mosiPin.pGPIOx = GPIOB;
 	mosiPin.pGPIOx_pin.GPIO_PinNumber = BIT15;
 	mosiPin.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_ALTFN;
 	mosiPin.pGPIOx_pin.GPIO_PinAltFunMode = BIT5;
 	mosiPin.pGPIOx_pin.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
 	mosiPin.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
 	mosiPin.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
 	GPIO_Init(&mosiPin);


 	// Configure SCLK Pin
 	GPIO_Handle_t sclkPin;
 	memset(&sclkPin, 0, sizeof(sclkPin));

 	sclkPin.pGPIOx = GPIOB;
 	sclkPin.pGPIOx_pin.GPIO_PinNumber = BIT13;
 	sclkPin.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_ALTFN;
 	sclkPin.pGPIOx_pin.GPIO_PinAltFunMode = BIT5;
 	sclkPin.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
 	sclkPin.pGPIOx_pin.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
 	sclkPin.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
 	GPIO_Init(&sclkPin);


 	// Configure SCLK Pin
	GPIO_Handle_t nssPin;
	memset(&nssPin, 0, sizeof(nssPin));

	nssPin.pGPIOx = GPIOB;
	nssPin.pGPIOx_pin.GPIO_PinNumber = BIT12;
	nssPin.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_ALTFN;
	nssPin.pGPIOx_pin.GPIO_PinAltFunMode = BIT5;
	nssPin.pGPIOx_pin.GPIO_PinSpeed = GPIO_SPEED_FAST;
	nssPin.pGPIOx_pin.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	nssPin.pGPIOx_pin.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	GPIO_Init(&nssPin);

}

void SPIinit()
{
	// Configure spi2 for master communication
	SPI_Handle_t spi2;
	memset(&spi2, 0, sizeof(spi2));

	spi2.pSPIx = SPI2;
	spi2.SPIConfig.SPI_BusConfig = SPI_BUS_CFG_FULLD;
	spi2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPIConfig.SPI_Speed = SPI_CLK_SPEED_DIV2;				// generates 8 MHz
	spi2.SPIConfig.SPI_DFF = SPI_DFF_MODE_8BIT;
	spi2.SPIConfig.SPI_CPHA = SPI_CLKPHA_LOW;
	spi2.SPIConfig.SPI_CPOL = SPI_CLKPOL_LOW;
	spi2.SPIConfig.SPI_SSM = SPI_SSM_DISABLE;
	SPI_Init(&spi2);
}

void ButtonInit()
{
	// Configure button to start TX
	GPIO_Handle_t config;
	config.pGPIOx = GPIOC;
	config.pGPIOx_pin.GPIO_PinNumber = 13;
	config.pGPIOx_pin.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Init(&config);
}

int main()
{
	ButtonInit();

	SPI_GPIOinit();

	SPIinit();

	// Manually setting SSI to high to avoid modf error.
//	SPI_SSOEConfig(SPI2, ENABLE);

	// Enable device after configuration
    SPI_SetupDevice(SPI2, ENABLE);

	// Send data
	char buff[] = "Hello World";
	SPI_SendData(SPI2, (uint8_t*)buff, strlen(buff));


// 	// Enable SSOE bit for hard NSS output enable
// 	SPI_SSOEConfig(SPI2, ENABLE);

// 	// Enable device after configuration
//     SPI_SetupDevice(SPI2, ENABLE);

// 	// Send data
// 	char buff[] = "Hello World";
// 	uint8_t dataLen = strlen(buff);

// 	// while (1)
// 	// {
// //		if (GPIO_ReadFromInputPin(GPIOC, 13) == 0) {

// 			// Delay(250000);
// 			SPI_SendData(SPI2, &dataLen, 1);

// 			// SPI_SendData(spi2.pSPIx, (uint8_t*)buff, strlen(buff));
// //			Delay(500000);
// //		}
// 	// }

	while (1);

// 	// Disable SPI2 peripheral
//     SPI_SetupDevice(SPI2, DISABLE);

	return 0;
}
