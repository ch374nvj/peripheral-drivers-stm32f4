/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Feb 13, 2025
 *      Author: Chetan
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

// 
// Configuration struct for a GPIO Pin
// 
typedef struct{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;               //Possible values - @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAFMode;
} GPIO_PinConfig_t;

// 
// Handle struct for a GPIO pin
// 
typedef struct{
    GPIO_RegDef_t *pGPIOx;              //Holds base add. of GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig;    //Holds GPIO pin config settings
} GPIO_Handle_t;

// 
// GPIO Macros
// 

// Pin Number
#define GPIO_PIN_0              0
#define GPIO_PIN_1              1
#define GPIO_PIN_2              2
#define GPIO_PIN_3              3
#define GPIO_PIN_4              4
#define GPIO_PIN_5              5
#define GPIO_PIN_6              6
#define GPIO_PIN_7              7
#define GPIO_PIN_8              8
#define GPIO_PIN_9              9
#define GPIO_PIN_10             10
#define GPIO_PIN_11             11
#define GPIO_PIN_12             12
#define GPIO_PIN_13             13
#define GPIO_PIN_14             14
#define GPIO_PIN_15             15

// @GPIO_PIN_MODES
// Mode
#define GPIO_MODE_INPUT         0
#define GPIO_MODE_OUTPUT        1
#define GPIO_MODE_AF            2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FT         4
#define GPIO_MODE_IT_RT         5
#define GPIO_MODE_IT_RFT        6

// Output Type
#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OD         1

// Speed
#define GPIO_OP_SPEED_LOW       0 
#define GPIO_OP_SPEED_MED       1
#define GPIO_OP_SPEED_FAST      2
#define GPIO_OP_SPEED_HI        3

// Pull-up/Pull-down
#define GPIO_PIN_NO_PUPD        0
#define GPIO_PIN_PULL_UP        1
#define GPIO_PIN_PULL_DOWN      2


/* =================== *
*   API Prototypes     *
*  =================== */

/// @brief GPIO Peripheral clock config
/// @param pGPIOx points to base address of gpio port (use GPIOA, GPIOB...)
/// @param Enable 1=> api enables gpio peripheral clk, 0=> disables it
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable); 

// GPIO Peripheral init and deinit(reset)

/// @brief api initializes GPIO pin, based on the PinConfig in *pGPIOHandle (pGPIOHandle->GPIO_PinConfig)
/// @param pGPIOHandle 
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/// @brief api to reset the given gpio port using RCC_AHB1RSTR 
/// @param pGPIOx 
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// GPIO Data Read and Write APIs

/// @brief GPIO Read (input)
/// @param pGPIOx pointer to base address of gpio port
/// @return The value at all 16 pins of the port
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/// @brief 
/// @param pGPIOx pointer to base address of gpio port
/// @param pinNumber the pin number of the port (0-15)
/// @return The state of selected pin 
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/// @brief GPIO Write (output)
/// @param pGPIOx pointer to base address of gpio port
/// @param pinNumber the pin number of the port (0-15)
/// @param Value SET(1) or RESET(0)
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);

/// @brief 
/// @param pGPIOx pointer to base address of gpio port
/// @param Value 16 bit value to be output at selected gpio port
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/// @brief API toggles the selected gpio pin
/// @param pGPIOx pointer to base address of gpio port
/// @param pinNumber the pin number of the port (0-15)
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

// IRQ Configuration and ISR Handling
// Following APIs deal at the processor side of MCU. So the stmts are processor specific.
// By processor side; it means we need to fiddle with the M4 processor's internal SFRs (Datasheet: Memory mapping)
// We would need to refer the Cortex-M4 processor's generic user guide to be able to write the API 

/// @brief API for IRQ Interrupt Enable
/// @param IRQNumber 
/// @param Enable 
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t Enable);

/// @brief Optional API for IRQ Priority setting
/// @param IRQNumber 
/// @param IRQPriority 0-15
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/// @brief This api clears the bit in exti pending reg (PR) corresp to pin number
/// @param PinNumber 
void GPIO_ISRHandling(uint8_t PinNumber);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
