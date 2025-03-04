/*
    SPI MOSI (output) Testing through logic analyzer
    Port: PA
    Peripheral: SPI1
    Alt. Fun: AF05
    NSS:  PA4
    SCK:  PA5
    MISO: PA6
    MOSI: PA7
*/

#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_gpio_driver.h"

void SPI1_GPIOInits(void) {
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
    SPIPins.GPIO_PinConfig.GPIO_PinAFMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIO_Init(&SPIPins);

    // SCK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&SPIPins);
}

void SPI1_Inits(void) {
    SPI_Handle_t SPI1handle;

    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPIConfig.SPI_BusConfig = SPI_FULL_DUPLEX;
    SPI1handle.SPIConfig.SPI_DeviceMode = SPI_MASTER;
    SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_PCLK_DIV2;
    SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI1handle.SPIConfig.SPI_SSM = ENABLE;

    SPI_Init(&SPI1handle);
    
    SPI_PeripheralEnable(SPI1, ENABLE);
    SPI_SSIConfig(SPI1, ENABLE);
}

int main(void) {
    char tx_data[] = "SPI MOSI Testing";

    SPI1_GPIOInits();
    SPI1_Inits();

    SPI_SendData(SPI2, (uint8_t*)tx_data, strlen(tx_data));

    SPI_PeripheralEnable(SPI1, DISABLE);

    while(1);

    return 0;
}