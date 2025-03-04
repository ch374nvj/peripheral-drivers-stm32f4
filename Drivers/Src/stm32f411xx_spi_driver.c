#include "stm32f411xx_spi_driver.h"
/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Chetan
 */

#include "stm32f411xx_spi_driver.h"

void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t Enable) {
    if(Enable){
        if(pSPIx==SPI1)
            SPI1_PCLK_EN();
        if(pSPIx==SPI2)
            SPI2_PCLK_EN();
        if(pSPIx==SPI3)
            SPI3_PCLK_EN();
        if(pSPIx==SPI4)
            SPI4_PCLK_EN();
        if(pSPIx==SPI5)
            SPI5_PCLK_EN();
    }
    else {
        if(pSPIx==SPI1)
            SPI1_PCLK_DI();
        if(pSPIx==SPI2)
            SPI2_PCLK_DI();
        if(pSPIx==SPI3)
            SPI3_PCLK_DI();
        if(pSPIx==SPI4)
            SPI4_PCLK_DI();
        if(pSPIx==SPI5)
            SPI5_PCLK_DI();
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {
    // Enable Peripheral Clock
    SPI_PClockControl(pSPIHandle->pSPIx, ENABLE);

    uint32_t tempCR1 = 0x00; // Temporary variable for setting control register

    // Setting Device Mode
    uint8_t deviceMode = pSPIHandle->SPIConfig.SPI_DeviceMode;
    tempCR1 |= (deviceMode<<2);

    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_FULL_DUPLEX) {
        tempCR1 &= ~(1<<SPI_CR1_BIDIMODE); 
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_HALF_DUPLEX) {
        tempCR1 |=  (1<<SPI_CR1_BIDIMODE);
    }
    else {
        tempCR1 &= ~(1<<SPI_CR1_BIDIMODE);
        tempCR1 |=  (1<<SPI_CR1_RXONLY);
    }

    // SCK Prescaler setting
    // tempCR1 &= ~(0x7 << BR); // Clear the BR bits before setting the required value in next line.
    tempCR1 |=  (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // CPOL, CPHA & SSM setting
    // tempCR1 &= ~(1 <<CPOL);
    tempCR1 |=  (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // tempCR1 &= ~(1<<CPHA);
    tempCR1 |=  (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    // tempCR1 &= ~(1<<SSM);
    tempCR1 |=  (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    pSPIHandle->pSPIx->CR[0] = tempCR1;

    /* Note: Clearing bit field isn't necessary when you are setting these bits into a
        temp variable.
    */
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    if(pSPIx==SPI1)
        SPI1_REG_RESET();
    if(pSPIx==SPI2)
        SPI2_REG_RESET();
    if(pSPIx==SPI3)
        SPI3_REG_RESET();
    if(pSPIx==SPI4)
        SPI4_REG_RESET();
    if(pSPIx==SPI5)
        SPI5_REG_RESET();
}

uint8_t SPI_GetBitStatus(SPI_RegDef_t *pSPIx, uint32_t BitName) {
    if (pSPIx->SR & (1<<BitName))
        return SET;
    else
        return RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
    while (Len > 0) {
        // while(!(pSPIx->SR & (1<<SPI_SR_TXE))); 
        // Above expression is a polling (blocking) call,
        // Whose equivalent is given below
        while(SPI_GetBitStatus(pSPIx, SPI_SR_TXE) == RESET); // Wait until Tx Buffer is empty (i.e. until TXE = 1)

        if(pSPIx->CR[0] & (1<<SPI_CR1_DFF)) { // 16 bit DFF?
            pSPIx->DR = *((uint16_t*)pTxBuffer); // Load data into DFF 
            Len = Len - 2;
            (uint16_t*)pTxBuffer++; // Increment addr of Tx Buffer
        }
        else {
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }

    }
    
}
