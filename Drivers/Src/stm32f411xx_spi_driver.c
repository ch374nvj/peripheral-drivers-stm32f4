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

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){

}
