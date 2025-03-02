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
#define SLAVE   0
#define MASTER  1

// @SPI_BusConfig
// BIDIMODE Bit; 0 => FD, 1 => HD
// BIDIOE   Bit, with BIDIMODE == 1; 0 => Rx-only mode (o/p disabled), 1 => Tx-only mode (o/p enabled)
// IF BIDIMODE == 0, BIDIOE = X
#define FULL_DUPLEX     0
#define HALF_DUPLEX     1
#define SIMPLEX_TXONLY  2
#define SIMPLEX_RXONLY  3

// @SPI_SclkSpeed
// Used in the BR register to adjust(prescale) the baud rate wrt Peripheral CLK (PCLK)
#define PCLK_DIV2       0
#define PCLK_DIV4       1
#define PCLK_DIV8       2
#define PCLK_DIV16      3
#define PCLK_DIV32      4
#define PCLK_DIV64      5
#define PCLK_DIV128     6
#define PCLK_DIV256     7

// @SPI_DFF
// Data Frame Format
#define DFF_8BITS       0
#define DFF_16BITS      1

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
#define SPI_SSM_SW      0
#define SPI_SSM_HW      1

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

// GPIO Data Read and Write APIs

/// @brief SPI Send (output)
/// @param pSPIx pointer to base address of SPI peripheral in use
/// @param pTxBuffer Pointer to Tx Buffer
/// @param Len Length (Size) of Tx data
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/// @brief SPI Receive (input)
/// @param *pSPIx pointer to base address of SPI peripheral in use
/// @param *pRxBuffer Pointer to Rx Buffer
/// @param Len Length (Size) of Rx data
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configuration and ISR Handling
// Following APIs deal at the processor side of MCU. So the stmts are processor specific.
// By processor side; it means we need to fiddle with the M4 processor's internal SFRs (Datasheet: Memory mapping)
// We would need to refer the Cortex-M4 processor's generic user guide to be able to write the API 

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

#endif /* STM32F411XX_SPI_DRIVER_H_ */
