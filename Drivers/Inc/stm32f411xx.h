/*
 * stm32f4xx.h
 *
 *  Created on: Feb 11, 2025
 *      Author: ch41c1d
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#include <stdint.h>
#define __vou volatile uint32_t

// 
// ARM Cortex M4 Processor Specific Details
// 

// NVIC ISERx Register addresses
#define NVIC_ISER(x)        (__vou*)(0xE000E100U + x*0x04)

// NVIC ICERx Register addresses
#define NVIC_ICER(x)        (__vou*)(0xE000E180U + x*0x04)

// NVIC IPRx Register address
#define NVIC_IPR            (__vou*)(0xE000E400U)

// 
// Base Addresses of Flash & SRAM
// 

#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM                SRAM1_BASEADDR
#define ROM                 0x1FFF0000U

// 
// Base addresses of different Bus domains
// 

#define PERIPH_BASEADDR     0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR (PERIPH_BASEADDR + 0x10000)
#define AHB1PERIPH_BASEADDR (PERIPH_BASEADDR + 0x20000)
#define AHB2PERIPH_BASEADDR (PERIPH_BASEADDR + 0x10000000)

// 
// Base addresses of required peripherals
// 

// RCC
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3800)

// SYSCFG
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800)

// GPIO
#define GPIOA_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0c00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1c00)

// I2C
#define I2C1_BASEADDR       (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR       (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR       (APB1PERIPH_BASEADDR + 0x5c00)

// SPI
#define SPI1_BASEADDR       (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR       (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR       (APB1PERIPH_BASEADDR + 0x3c00)
#define SPI4_BASEADDR       (APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR       (APB2PERIPH_BASEADDR + 0x5000)

// USART
#define USART1_BASEADDR     (APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR     (APB1PERIPH_BASEADDR + 0x4400)
#define USART6_BASEADDR     (APB2PERIPH_BASEADDR + 0x1400)

// EXTI
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00)

// 
// Peripheral Register structures
// 

// RCC
typedef struct {
    __vou CR;
    __vou PLLCFGR;
    __vou CFGR;
    __vou CIR;
    __vou AHB1RSTR;
    __vou AHB2RSTR;
    __vou RESERVED0[2];
    __vou APB1RSTR;
    __vou APB2RSTR;
    __vou RESERVED1[2];
    __vou AHB1ENR;
    __vou AHB2ENR;
    __vou RESERVED2[2];
    __vou APB1ENR;
    __vou APB2ENR;
    __vou RESERVED3[2];
    __vou AHB1LPENR;
    __vou AHB2LPENR;
    __vou RESERVED4[2];
    __vou APB1LPENR;
    __vou APB2LPENR;
    __vou RESERVED5[2];
    __vou BDCR;
    __vou CSR;
    __vou RESERVED6[2];
    __vou SSCGR;
    __vou PLLI2SCFGR;
    __vou RESERVED7;
    __vou DCKCFGR;
} RCC_RegDef_t;

// SYSCFG
typedef struct {
    __vou MEMRMP;
    __vou PMC;
    __vou EXTICR[4];
    __vou RESERVED0[2];
    __vou CMPCR;
}SYSCFG_RegDef_t;

// GPIO
typedef struct {
    volatile uint32_t MODER;
    __vou OTYPER;
    __vou OSPEEDR;
    __vou PUPDR;
    __vou IDR;
    __vou ODR;
    __vou BSRR;
    __vou LCKR;
    __vou AFR[2]; // AFR[0] -> AFRL, AFR[1] -> AFRH
} GPIO_RegDef_t;

// I2C
typedef struct {
    __vou CR1;
    __vou CR2;
    __vou OAR1;
    __vou OAR2;
    __vou DR;
    __vou SR1;
    __vou SR2;
    __vou CCR;
    __vou TRISE;
    __vou FLTR;
} I2C_RegDef_t;

// SPI
typedef struct {
    __vou CR[2]; // CR[0] -> CR1, CR[1] -> CR2
    __vou SR;
    __vou DR;
    __vou CRCPR;
    __vou RXCRCR;
    __vou TXCRCR;
    __vou I2SCFGR;
    __vou I2SPR;
} SPI_RegDef_t;

// USART
typedef struct {
    __vou SR;
    __vou DR;
    __vou BRR;
    __vou CR1;
    __vou CR2;
    __vou CR3;
    __vou GTPR;
} USART_RegDef_t;

// EXTI
typedef struct {
    __vou IMR;
    __vou EMR;
    __vou RTSR;
    __vou FTSR;
    __vou SWIER;
    __vou PR;
} EXTI_RegDef_t;

// 
// Peripheral definitions
// 

// RCC
#define RCC                 ((RCC_RegDef_t*)RCC_BASEADDR)

// SYSCFG
#define SYSCFG              ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

// EXTI
#define EXTI                ((EXTI_RegDef_t*)EXTI_BASEADDR)

// GPIO (@pGPIOx)
#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH               ((GPIO_RegDef_t*)GPIOH_BASEADDR)

// I2C
#define I2C1                ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                ((I2C_RegDef_t*)I2C3_BASEADDR)

// SPI (@pSPIx)
#define SPI1                ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5                ((SPI_RegDef_t*)SPI5_BASEADDR)

// USART
#define USART1              ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2              ((USART_RegDef_t*)USART2_BASEADDR)
#define USART6              ((USART_RegDef_t*)USART6_BASEADDR)

// 
// Clock enable macros
// 

// SYSCFG
#define SYSCFG_PCLK_EN()    RCC->APB2ENR |= (1<<14)

// GPIO
#define GPIOA_PCLK_EN()     RCC->AHB1ENR |= (1<<0)
#define GPIOB_PCLK_EN()     RCC->AHB1ENR |= (1<<1)
#define GPIOC_PCLK_EN()     RCC->AHB1ENR |= (1<<2)
#define GPIOD_PCLK_EN()     RCC->AHB1ENR |= (1<<3)
#define GPIOE_PCLK_EN()     RCC->AHB1ENR |= (1<<4)
#define GPIOH_PCLK_EN()     RCC->AHB1ENR |= (1<<7)

// I2C
#define I2C1_PCLK_EN()      RCC->APB1ENR |= (1<<21)
#define I2C2_PCLK_EN()      RCC->APB1ENR |= (1<<22)
#define I2C3_PCLK_EN()      RCC->APB1ENR |= (1<<23)

// SPI
#define SPI1_PCLK_EN()      RCC->APB2ENR |= (1<<12)
#define SPI2_PCLK_EN()      RCC->APB1ENR |= (1<<14)
#define SPI3_PCLK_EN()      RCC->APB1ENR |= (1<<15)
#define SPI4_PCLK_EN()      RCC->APB2ENR |= (1<<13)
#define SPI5_PCLK_EN()      RCC->APB2ENR |= (1<<20)

// USART
#define USART1_PCLK_EN()    RCC->APB2ENR |= (1<<4)
#define USART2_PCLK_EN()    RCC->APB1ENR |= (1<<17)
#define USART6_PCLK_EN()    RCC->APB2ENR |= (1<<5)

// 
// Clock disable macros
// 

// GPIO
#define GPIOA_PCLK_DI()     RCC->AHB1ENR &= ~(1<<0)
#define GPIOB_PCLK_DI()     RCC->AHB1ENR &= ~(1<<1)
#define GPIOC_PCLK_DI()     RCC->AHB1ENR &= ~(1<<2)
#define GPIOD_PCLK_DI()     RCC->AHB1ENR &= ~(1<<3)
#define GPIOE_PCLK_DI()     RCC->AHB1ENR &= ~(1<<4)
#define GPIOH_PCLK_DI()     RCC->AHB1ENR &= ~(1<<7)

// I2C
#define I2C1_PCLK_DI()      RCC->APB1ENR &= ~(1<<21)
#define I2C2_PCLK_DI()      RCC->APB1ENR &= ~(1<<22)
#define I2C3_PCLK_DI()      RCC->APB1ENR &= ~(1<<23)

// SPI
#define SPI1_PCLK_DI()      RCC->APB2ENR &= ~(1<<12)
#define SPI2_PCLK_DI()      RCC->APB1ENR &= ~(1<<14)
#define SPI3_PCLK_DI()      RCC->APB1ENR &= ~(1<<15)
#define SPI4_PCLK_DI()      RCC->APB2ENR &= ~(1<<13)
#define SPI5_PCLK_DI()      RCC->APB2ENR &= ~(1<<20)

// USART
#define USART1_PCLK_DI()    RCC->APB2ENR &= ~(1<<4)
#define USART2_PCLK_DI()    RCC->APB1ENR &= ~(1<<17)
#define USART6_PCLK_DI()    RCC->APB2ENR &= ~(1<<5) 

// 
// GPIOx Reset Macros
// 
#define GPIOA_REG_RESET()   do{RCC->AHB1RSTR |= (1<<0); (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()   do{RCC->AHB1RSTR |= (1<<1); (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()   do{RCC->AHB1RSTR |= (1<<2); (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()   do{RCC->AHB1RSTR |= (1<<3); (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()   do{RCC->AHB1RSTR |= (1<<4); (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOH_REG_RESET()   do{RCC->AHB1RSTR |= (1<<7); (RCC->AHB1RSTR &= ~(1<<7));} while(0)

// GPIOx -> number Macro
#define GPIO_BASE_TO_CODE(x)    (x==GPIOA)? 0: \
                                (x==GPIOB)? 1: \
                                (x==GPIOC)? 2: \
                                (x==GPIOD)? 3: \
                                (x==GPIOE)? 4: \
                                (x==GPIOH)? 7: 0

// 
// SPIx Reset Macros
// 
#define SPI1_REG_RESET()    do{ }

// IRQ Number Macros
// Postion = IRQ Number (Ref Manual: Vector table)
#define IRQ_EXTI0           6
#define IRQ_EXTI1           7
#define IRQ_EXTI2           8
#define IRQ_EXTI3           9
#define IRQ_EXTI4           10
#define IRQ_EXTI9_5         23
#define IRQ_EXTI15_10       40

// General Macros
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE

// #include "stm32f411xx_gpio_driver.h"
// #include "stm32f411xx_spi_driver.h"

#endif /* STM32F411XX_H_ */
