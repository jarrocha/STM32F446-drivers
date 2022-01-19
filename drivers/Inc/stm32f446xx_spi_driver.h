/***********************************************
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Nov 3, 2021
 *      Author: Jaime Arrocha
 ***********************************************/

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/***********************************************
 * Configuration structure for SPIx peripheral
 ***********************************************/
typedef struct {
    uint8_t SPI_DeviceMode;         // Master or slave selection
    uint8_t SPI_BusConfig;          // Communication type: Duplex, Half-Duplex, Simplex
    uint8_t SPI_Speed;              // Baud rate
    uint8_t SPI_DFF;                // Data frame format: 8-bit, 16-bit transmission/receival
    uint8_t SPI_CPOL;               // Default value 0
    uint8_t SPI_CPHA;               // Default value 0
    uint8_t SPI_SSM;                // Slave-Select management
} SPI_Config_t;

/***********************************************
 * Handle structure for SPIx peripheral
 ***********************************************/
typedef struct {
    SPI_RegDef_t* pSPIx;            // Holds base address of peripheral
    SPI_Config_t SPIConfig;
} SPI_Handle_t;

/***********************************************
 * SPI Configuration helpers
 ***********************************************/
#define SPI_DEVICE_MODE_SLAVE       0
#define SPI_DEVICE_MODE_MASTER      1

#define SPI_BUS_CFG_HALFD           0
#define SPI_BUS_CFG_FULLD           1
#define SPI_BUS_CFG_SIMPLEX_RX      2

#define SPI_CLK_SPEED_DIV2          0
#define SPI_CLK_SPEED_DIV4          1
#define SPI_CLK_SPEED_DIV8          2
#define SPI_CLK_SPEED_DIV16         3
#define SPI_CLK_SPEED_DIV32         4
#define SPI_CLK_SPEED_DIV64         5
#define SPI_CLK_SPEED_DIV128        6
#define SPI_CLK_SPEED_DIV256        7

#define SPI_DFF_MODE_8BIT           0
#define SPI_DFF_MODE_16BIT          1

#define SPI_CLKPOL_LOW              0
#define SPI_CLKPOL_HIGH             1

#define SPI_CLKPHA_LOW              0
#define SPI_CLKPHA_HIGH             1

#define SPI_SSM_DISABLE             0
#define SPI_SSM_ENABLE              1

/***********************************************
 * SPI Control Registers 1, Bit Definitions
 ***********************************************/
#define SPI_CR1_CPHA                0
#define SPI_CR1_CPOL                1
#define SPI_CR1_MSTR                2
#define SPI_CR1_BR                  3
#define SPI_CR1_SPE                 6
#define SPI_CR1_LSBFIRST            7
#define SPI_CR1_SSI                 8
#define SPI_CR1_SSM                 9
#define SPI_CR1_RXONLY              10
#define SPI_CR1_DFF                 11
#define SPI_CR1_CRCNEXT             12
#define SPI_CR1_CRCEN               13
#define SPI_CR1_BIDIOE              14
#define SPI_CR1_BIDIMODE            15

/***********************************************
 * SPI Control Registers 2, Bit Definitions
 ***********************************************/
#define SPI_CR2_RXDMAEN             0
#define SPI_CR2_TXDMAEN             1
#define SPI_CR2_SSOE                2
#define SPI_CR2_FRF                 4
#define SPI_CR2_ERRIE               5
#define SPI_CR2_RXNEIE              6
#define SPI_CR2_TXEIE               7

/***********************************************
 * SPI Status Register, Bit Definitions
 ***********************************************/
#define SPI_SR_RXNE                 0
#define SPI_SR_TXE                  1
#define SPI_SR_CHSIDE               2
#define SPI_SR_UDR                  3
#define SPI_SR_CRCERR               4
#define SPI_SR_MODF                 5
#define SPI_SR_OVR                  6
#define SPI_SR_BSY                  7
#define SPI_SR_FRE                  8

/***********************************************
 * 	 Peripheral Enable
 ***********************************************/
#define SPI1_DEV_EN()		(SPI1->SPI_CR1 |= (1 << SPI_CR1_SPE))
#define SPI2_DEV_EN()		(SPI2->SPI_CR1 |= (1 << SPI_CR1_SPE))
#define SPI3_DEV_EN()		(SPI3->SPI_CR1 |= (1 << SPI_CR1_SPE))
#define SPI4_DEV_EN()		(SPI4->SPI_CR1 |= (1 << SPI_CR1_SPE))

/***********************************************
 * 	 Peripheral Clock Setup
 ***********************************************/
int32_t SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t enableState);

/************************************************
 * 	 Init and reset GPIO port
 ************************************************/
int32_t SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_Handle_t* pSPIHandle);

/************************************************
 * 	 SSI Handling
 ************************************************/

int32_t SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t enableState);

// Controls the SSOE bit in SPI_CR2. This sets NSS output to enable/disable.
int32_t SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t enableState);

/************************************************
 * 	 Data Send and Receive
 ************************************************/
// Synchronous send. Returns size of data send, -1 on error.
int32_t SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t len);

// Synchronous receive. Returns size of recieved data.
int32_t SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t len);

/************************************************
 * 	 IRQ Configuration and ISR Handling
 ************************************************/
void SPI_IRQConfig(uint8_t IRQnumber, uint8_t enableState);
void SPI_IRQPriorityConfig(uint8_t IRQnumber, uint32_t IRQPriority);
void SPI_IRQHandle(SPI_Handle_t* pHandle);

/************************************************
 * 	 Other Peripheral Control APIs
 ************************************************/

// Enables and disables SPI_CR1_SPE for peripheral enable/disable
int32_t SPI_SetupDevice(SPI_RegDef_t* pSPIx, uint8_t enableState);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
