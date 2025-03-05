/*
    SPI Master & Slave command execution code
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

// Command Code Macros
#define COMMAND_LED_CTRL        0X50
#define COMMAND_SENSOR_READ     0X51
#define COMMAND_LED_READ        0X52
#define COMMAND_PRINT           0X53
#define COMMAND_ID_READ         0X54

#define LED_PIN                 9
#define LED_OFF                 0
#define LED_ON                  1

#define SENSOR_PIN              5

void delay(void) {
    for(uint32_t i = 0; i < 50000/2; i++);
}

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
    
    SPI_SSIConfig(SPI1, ENABLE);
}

uint8_t SPI_VerifyResponse(uint8_t ack) {
    if (ack == 0xF5) // 0xF5 value for ACK response defined by us in the slave
        return 1;
    return 0;
}

GPIO_ButtonInit(void) {
    GPIO_Handle_t buttonGPIO;
    memset(&buttonGPIO, 0, sizeof(buttonGPIO));
    buttonGPIO.pGPIOx = GPIOA;
    buttonGPIO.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_INPUT;
    buttonGPIO.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_0;
    buttonGPIO.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
    buttonGPIO.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
    buttonGPIO.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SPEED_LOW;

    GPIO_PClockControl(GPIOA, ENABLE);
    GPIO_Init(&buttonGPIO);
}

int main(void) {
    uint8_t dummy_byte = 0xFF;
    uint8_t dummy_read;

    SPI1_GPIOInits();
    SPI1_Inits();
    SPI_SSOEConfig(SPI1, ENABLE);
    GPIO_ButtonInit();

    while (1) {
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); // wait till button pressed
        delay(); // To avoid debouncing

        SPI_PeripheralEnable(SPI1, ENABLE);

        // 1. CMD_LED_CTRL  <pin_no>    <value>

        uint8_t command_code = COMMAND_LED_CTRL;
        uint8_t ack;
        uint8_t args[2];
        SPI_SendData(SPI1, &command_code, 1);

        // A dummy read operation to clear the Rx Buffer and reset RXNE bit
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Sending dummy byte to be able to recieve response from slave
        SPI_SendData(SPI1, &dummy_byte, 1);
        SPI_ReceiveData(SPI1, &ack, 1);

        if(SPI_VerifyResponse(ack)) {
            // Proceed to set command args
            args[0] = SENSOR_PIN;

            // Tx of command args
            SPI_SendData(SPI1, args, 2);
        }

        // 2. CMD_SENSOR_READ   <analog_pin_no (1B)>
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); // wait till button pressed
        delay(); // To avoid debouncing

        command_code = COMMAND_SENSOR_READ;
        SPI_SendData(SPI1, &command_code, 1);

        // A dummy read operation to clear the Rx Buffer and reset RXNE bit
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Sending dummy byte to be able to recieve response from slave
        SPI_SendData(SPI1, &dummy_byte, 1);
        SPI_ReceiveData(SPI1, &ack, 1);

        if(SPI_VerifyResponse(ack)) {
            // Proceed to set command args
            args[0] = SENSOR_PIN;
            // Tx of command args
            SPI_SendData(SPI1, args, 1);
            SPI_ReceiveData(SPI1, &dummy_read, 1); // A dummy read to reset RXNE bit
            
            delay(); // A small delay for the master to wait for a safe amt of time for the slave to read sensor data and 
            // perform some ADC converions and then put the same in its Tx Buffer 
            
            // Sending dummy byte to be able to recieve response from slave
            uint8_t sensor_read;
            SPI_SendData(SPI1, &dummy_byte, 1);
            SPI_ReceiveData(SPI1, &sensor_read, 1);
        }

        // 3. COMMAND_LED_READ   <pin_no (1B)>
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); // wait till button pressed
        delay(); // To avoid debouncing

        command_code = COMMAND_LED_READ;
        SPI_SendData(SPI1, &command_code, 1);

        // A dummy read operation to clear the Rx Buffer and reset RXNE bit
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Sending dummy byte to be able to recieve response from slave
        SPI_SendData(SPI1, &dummy_byte, 1);
        SPI_ReceiveData(SPI1, &ack, 1);

        if(SPI_VerifyResponse(ack)) {
            // Proceed to set command args
            args[0] = LED_PIN;
            // Tx of command args
            SPI_SendData(SPI1, args, 1);
            SPI_ReceiveData(SPI1, &dummy_read, 1); // A dummy read to reset RXNE bit
            
            delay(); // A small delay for the master to wait for a safe amt of time for the slave to be ready to send data 
            
            // Sending dummy byte to be able to recieve response from slave
            uint8_t led_read;
            SPI_SendData(SPI1, &dummy_byte, 1);
            SPI_ReceiveData(SPI1, &led_read, 1);
        }

        // 4. COMMAND_PRINT   <len (2B) message(len)>
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); // wait till button pressed
        delay(); // To avoid debouncing

        command_code = COMMAND_PRINT;
        SPI_SendData(SPI1, &command_code, 1);

        // A dummy read operation to clear the Rx Buffer and reset RXNE bit
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Sending dummy byte to be able to recieve response from slave
        SPI_SendData(SPI1, &dummy_byte, 1);
        SPI_ReceiveData(SPI1, &ack, 1);

        uint8_t message[] = 'Hello SPI';
        if(SPI_VerifyResponse(ack)) {
            // Proceed to set command args
            args[0] = strlen((char *)message);
            // Tx of command args
            SPI_SendData(SPI1, args, 1);
            SPI_ReceiveData(SPI1, &dummy_read, 1); // A dummy read to reset RXNE bit
            
            SPI_SendData(SPI1, &message, args[0]);
            SPI_ReceiveData(SPI1, &dummy_read, 1);
        }

        // 5. COMMAND_ID_READ   
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); // wait till button pressed
        delay(); // To avoid debouncing

        command_code = COMMAND_ID_READ;
        SPI_SendData(SPI1, &command_code, 1);

        // A dummy read operation to clear the Rx Buffer and reset RXNE bit
        SPI_ReceiveData(SPI1, &dummy_read, 1);

        // Sending dummy byte to be able to recieve response from slave
        SPI_SendData(SPI1, &dummy_byte, 1);
        SPI_ReceiveData(SPI1, &ack, 1);

        uint8_t id[10];
        uint32_t i = 0;
        if(SPI_VerifyResponse(ack)) {
            for (i=0; i< 10; i++) {
                SPI_SendData(SPI1, &dummy_byte, 1);
                SPI_ReceiveData(SPI1, &id[i], 1);
            }
            id[11] = '\0';
        }

        while (SPI_GetBitStatus(SPI1, SPI_SR_BSY)); // Wait until SPI not busy 
        
        SPI_PeripheralEnable(SPI1, DISABLE);
    }

    while(1);

    return 0;
}