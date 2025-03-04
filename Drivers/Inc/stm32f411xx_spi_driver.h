/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Feb 27, 2025
 *      Author: Chetan
 */

#ifndef STM32F411XX_SPI_DRIVER_H_
#define STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

//
// Config Struct for SPIx Peripheral 
// 
typedef struct {
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

// 
// Handle struct for SPIx Peripheral
// 
typedef struct {
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
} SPI_Handle_t;

/* =================== *
*   SPI Macros         *
*  =================== */

// @SPI_DeviceMode
// CR1 MSTR Bit
#define SPI_SLAVE   0
#define SPI_MASTER  1

// @SPI_BusConfig
// BIDIMODE Bit; 0 => FD, 1 => HD
// BIDIOE   Bit, with BIDIMODE == 1; 0 => Rx-only mode (o/p disabled), 1 => Tx-only mode (o/p enabled)
// IF BIDIMODE == 0, BIDIOE = X
// IF RXONLY == 1, SIMPLEX_RXONLY Mode 
#define SPI_FULL_DUPLEX     0
#define SPI_HALF_DUPLEX     1
#define SPI_SIMPLEX_RXONLY  2

// @SPI_SclkSpeed
// Used in the BR register to adjust(prescale) the baud rate wrt Peripheral CLK (PCLK)
#define SPI_PCLK_DIV2       0
#define SPI_PCLK_DIV4       1
#define SPI_PCLK_DIV8       2
#define SPI_PCLK_DIV16      3
#define SPI_PCLK_DIV32      4
#define SPI_PCLK_DIV64      5
#define SPI_PCLK_DIV128     6
#define SPI_PCLK_DIV256     7

// @SPI_DFF
// Data Frame Format
// DFF Bit in CR1
#define SPI_DFF_8BITS       0
#define SPI_DFF_16BITS      1

// @SPI_CPOL
// Clock polarity
#define SPI_CPOL_LOW    0
#define SPI_CPOL_HIGH   1

// @SPI_CPHA
// Clock Phase
#define SPI_CPHA_LOW    0
#define SPI_CPHA_HIGH   1

// @SPI_SSM
// Slave Select Mgmt
#define SPI_SSM_DI      0
#define SPI_SSM_EN      1

// SPI_CR1 Register Bit Positions
#define SPI_CR1_BIDIMODE        15
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_CRCEN           13
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_DFF             11
#define SPI_CR1_RXONLY          10
#define SPI_CR1_SSM             9
#define SPI_CR1_SPE             6
#define SPI_CR1_BR              3
#define SPI_CR1_MSTR            2
#define SPI_CR1_CPOL            1
#define SPI_CR1_CPHA            0

// SPI_CR2 Register Bit Positions
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7

// SPI_SR Register Bit Positions
#define SPI_SR_RXNE                    0
#define SPI_SR_TXE                     1
#define SPI_SR_CHSIDE                  2                
#define SPI_SR_UDR                     3        
#define SPI_SR_CRCERR                  4            
#define SPI_SR_MODF                    5            
#define SPI_SR_OVR                     6        
#define SPI_SR_BSY                     7        
#define SPI_SR_FRE                     8

/* =================== *
*   API Prototypes     *
*  =================== */

/// @brief SPI Peripheral clock config
/// @param pSPIx points to base address of SPI Peripheral (use SPI1, SPI2...)
/// @param Enable 1=> api enables SPI peripheral clk, 0=> disables it
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t Enable);

// SPI Peripheral init and deinit(reset)

/// @brief api initializes SPI peripheral, based on the PinConfig in *pSPIHandle (pSPIHandle->SPI_PinConfig)
/// @param pSPIHandle 
void SPI_Init(SPI_Handle_t *pSPIHandle);

/// @brief api to reset the given gpio port using RCC_AHB1RSTR 
/// @param pSPIx 
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/// @brief Sets/Resets the SPE Bit in Control register to Enable/Disable the SPI Peripheral 
/// @param pSPIx
/// @param Enable Enable or Disable the peripheral
void SPI_PeripheralEnable(SPI_RegDef_t *pSPIx, uint8_t Enable);

// GPIO Data Read and Write APIs

/// @brief SPI Send (output)
/// @param pSPIx pointer to base address of SPI peripheral in use
/// @param pTxBuffer Pointer to Tx Buffer
/// @param Len Length (Size) of Tx data in bytes
/// @note A blocking API, i.e., waits until all bytes in the buffer are sent.
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/// @brief SPI Receive (input)
/// @param *pSPIx pointer to base address of SPI peripheral in use
/// @param *pRxBuffer Pointer to Rx Buffer
/// @param Len Length (Size) of Rx data
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/// @note  Configuration and ISR Handling:
/// Following APIs deal at the processor side of MCU. So the stmts are processor specific.
/// By processor side; it means we need to fiddle with the M4 processor's internal SFRs (Datasheet: Memory mapping)
/// We would need to refer the Cortex-M4 processor's generic user guide to be able to write the API 

/// @brief API for IRQ Interrupt Enable
/// @param IRQNumber 
/// @param Enable 
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t Enable);

/// @brief Optional API for IRQ Priority setting
/// @param IRQNumber 
/// @param IRQPriority 0-15
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/// @brief This api clears the bit in exti pending reg (PR) corresp to pin number
/// @param PinNumber 
void SPI_ISRHandling(uint8_t PinNumber);

// Misc APIs

/// @brief A wrapper to access and read the status bits from the SPI_SR Reg
/// @param pSPIx 
/// @param BitName 
/// @return SET/RESET (int 1/0)
uint8_t SPI_GetBitStatus(SPI_RegDef_t *pSPIx, uint32_t BitName);

/// @brief Sets/Resets the SSI Bit in Control register 
/// @param pSPIx
/// @param Set Set or reset the bit
/// @note Reference Manual 20.3.10
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Set);

#endif /* STM32F411XX_SPI_DRIVER_H_ */
