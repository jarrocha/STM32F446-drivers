/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Nov 3, 2021
 *      Author: jaime
 */

#include <stdio.h>
#include "stm32f446xx_spi_driver.h"

int32_t SPI_SetupDevice(SPI_RegDef_t* pSPIx, uint8_t enableState);

int32_t SPI_Init(SPI_Handle_t* pSPIHandle)
{
    // Pointer for SPI Control Register 1
    uint32_t tmp = 0;

    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // Configure master/slave mode
    tmp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

    // Configure bus mode. Can be Duplex, Half-Duplex, and Simplex.
    switch (pSPIHandle->SPIConfig.SPI_BusConfig)
    {
        case SPI_BUS_CFG_FULLD:
        {
            // This is the default mode. No config needed. Left for reference.
            tmp &= ~(1 << SPI_CR1_BIDIMODE);         // Bi-directional mode cleared
            break;
        }
        case SPI_BUS_CFG_HALFD:
        {
            tmp |= (1 << SPI_CR1_BIDIMODE);          // Bi-directional mode set
            break;
        }
        case SPI_BUS_CFG_SIMPLEX_RX:
        {
            tmp &= ~(1 << SPI_CR1_BIDIMODE);         // Bi-directional mode cleared
            tmp |= (1 << SPI_CR1_RXONLY);            // RXONlY must be set
            break;
        }
        default:
        {
            // TODO: improve logging for errors
            return -1;
        }
    }

    // Configure speed
    tmp |= (pSPIHandle->SPIConfig.SPI_Speed << SPI_CR1_BR);

    // Configure data frame format
    tmp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

    // Configure clock polarity
    tmp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // Configure clock phase
    tmp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    // Configure slave select management
    tmp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    pSPIHandle->pSPIx->SPI_CR1 = tmp;

    return 0;
}

void SPI_DeInit(SPI_Handle_t* pSPIHandle)
{

}

int32_t SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t enableState)
{
    if (pSPIx == NULL) {
		printf("LOG: error: invalid SPI port.\n");
		return -1;
	}

    if (enableState) {
        (pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI));
    } else {
        (pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI));
    }

    return 0;
}

int32_t SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t enableState)
{
    if (pSPIx == NULL) {
		printf("LOG: error: invalid SPI port.\n");
		return -1;
	}

    if (enableState) {
        (pSPIx->SPI_CR1 |= (1 << SPI_CR2_SSOE));
    } else {
        (pSPIx->SPI_CR1 &= ~(1 << SPI_CR2_SSOE));
    }

    return 0;
}

int32_t SPI_SetupDevice(SPI_RegDef_t* pSPIx, uint8_t enableState)
{
    if (pSPIx == NULL) {
		printf("LOG: error: invalid SPI port.\n");
		return -1;
	}

    if (enableState) {
        if (pSPIx == SPI1) {
            SPI1_DEV_EN();
        } else if (pSPIx == SPI2) {
            SPI2_DEV_EN();
        } else if (pSPIx == SPI3) {
            SPI3_DEV_EN();
        } else if (pSPIx == SPI4) {
            SPI4_DEV_EN();
        } else {
            printf("LOG: error: invalid SPI port.\n");
            return -1;
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_DI();
        } else if (pSPIx == SPI4) {
            SPI4_PCLK_DI();
        } else {
            printf("LOG: error: invalid SPI port.\n");
            return -1;
        }
    }

    return 0;
}

int32_t SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t enableState)
{
    if (pSPIx == NULL) {
		printf("LOG: error: invalid SPI port.\n");
		return -1;
	}

    if (enableState) {
        if (pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        } else if (pSPIx == SPI4) {
            SPI4_PCLK_EN();
        } else {
            printf("LOG: error: invalid SPI port.\n");
            return -1;
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_DI();
        } else if (pSPIx == SPI4) {
            SPI4_PCLK_DI();
        } else {
            printf("LOG: error: invalid SPI port.\n");
            return -1;
        }
    }

    return 0;
}

void SPI_IRQConfig(uint8_t IRQnumber, uint8_t enableState)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQnumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandle(SPI_Handle_t* pHandle)
{

}

int32_t SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
    // if (len == 0)
    //     return -1;

    while (len > 0) {
        // TODO: May hang. Enable watchdog for this.
        
        // TXE (SPI_SR_TXE) is set when TX buffer is empty and ready for TX.
        // Wait while TXE (SPI_SR_TXE) is not set.
        while (TestBit(pSPIx->SPI_SR, SPI_SR_TXE) == 0);

        // Check frame format
        if (TestBit(pSPIx->SPI_CR1, SPI_CR1_DFF) == SPI_DFF_MODE_8BIT) {

            pSPIx->SPI_DR = *pTxBuffer;     // load data register
            len--;                          // decrement len
            pTxBuffer++;                    // increment pointer offset

        } else if (TestBit(pSPIx->SPI_CR1, SPI_CR1_DFF) == SPI_DFF_MODE_16BIT) {

            pSPIx->SPI_DR = *((uint16_t*) pTxBuffer);       // load data register
            len -= 2;                                       // decrement len
            (uint16_t*) pTxBuffer++;                        // increment pointer offset
        } else {
            // TODO: wrong data format error
            return -1;
        }
    }

    return 0;
}

int32_t SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
    return 0;
}
