/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Feb 13, 2025
 *      Author: Chetan
 */

#include "stm32f411xx_gpio_driver.h"

void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable){
    if(Enable){
        if(pGPIOx==GPIOA)
            GPIOA_PCLK_EN();
        if(pGPIOx==GPIOB)
            GPIOB_PCLK_EN();
        if(pGPIOx==GPIOC)
            GPIOC_PCLK_EN();
        if(pGPIOx==GPIOD)
            GPIOD_PCLK_EN();
        if(pGPIOx==GPIOE)
            GPIOE_PCLK_EN();
        if(pGPIOx==GPIOH)
            GPIOH_PCLK_EN();
    }
    else {
        if(pGPIOx==GPIOA)
            GPIOA_PCLK_DI();
        if(pGPIOx==GPIOB)
            GPIOB_PCLK_DI();
        if(pGPIOx==GPIOC)
            GPIOC_PCLK_DI();
        if(pGPIOx==GPIOD)
            GPIOD_PCLK_DI();
        if(pGPIOx==GPIOE)
            GPIOE_PCLK_DI();
        if(pGPIOx==GPIOH)
            GPIOH_PCLK_DI();
    }
} 


void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
    // Peripheral Clock Enable
    GPIO_PClockControl(pGPIOHandle->pGPIOx, ENABLE);
    // Mode Config
    uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    uint8_t pinMode   = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    if(pinMode == GPIO_MODE_INPUT || pinMode == GPIO_MODE_OUTPUT){
        pGPIOHandle->pGPIOx->MODER &=    ~(0x3 << (2*pinNumber)); // Clear bit field
        pGPIOHandle->pGPIOx->MODER |= (pinMode << (2*pinNumber));
        // 2*pinNumber => Because MODER allocates 2 bits for each gpio pin, so offset 
        // is twice for each pin number, refer 8.4.1(Ref Manual)
    }
    else{
        if(pinMode == GPIO_MODE_IT_FT){
            EXTI->FTSR |=  (1<<pinNumber);
            EXTI->RTSR &= ~(1<<pinNumber);
            // For falling edge trigger, the Bit corresponding to pin number (for eg:4),
            // the 4th bit of FTSR is set, while the 4th bit of RTSR is reset
        }
        else if(pinMode == GPIO_MODE_IT_RT){
            EXTI->FTSR &= ~(1<<pinNumber);
            EXTI->RTSR |=  (1<<pinNumber);
            // For rising edge trigger, the Bit corresponding to pin number (for eg:4),
            // the 4th bit of RTSR is set, while the 4th bit of FTSR is reset
        }
        else if(pinMode == GPIO_MODE_IT_RFT){
            EXTI->FTSR |=  (1<<pinNumber);
            EXTI->RTSR |=  (1<<pinNumber);
            // For all edge trigger, the Bit corresponding to pin number (for eg:4),
            // the 4th bit of FTSR is set, and the 4th bit of RTSR is also set
        }
        // SYSCFG: to map the EXTI line to a pin of Req. GPIO "PORT"
        // EXTI lines are shared among all Ports, but can deliver an interrupt only from the
        // PORTx specified in software
        uint8_t portCode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
        uint8_t idx      = pinNumber / 4;
        uint8_t offset   =(pinNumber % 4)*4;
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[idx] = portCode << offset;

        // IMR: to allow delivery of interrupt from selected pin, by unmasking that pin number
        EXTI->IMR |= 1 << pinNumber;
    }
    // Speed Config
    uint8_t pinSpeed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
    pGPIOHandle->pGPIOx->OSPEEDR &=     ~(0x3 << (2*pinNumber)); // Clear bit field
    pGPIOHandle->pGPIOx->OSPEEDR |= (pinSpeed << (2*pinNumber));
    // 2*pinNumber => Because OSPEEDR allocates 2 bits for each gpio pin

    // PUPD Config
    uint8_t puPd = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= (puPd << (2*pinNumber));

    // Output type Config
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT) {
        uint8_t pinOutputType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
        pGPIOHandle->pGPIOx->OTYPER &=          ~(0x1 << pinNumber);
        pGPIOHandle->pGPIOx->OTYPER |= (pinOutputType << pinNumber);
    }

    // Alternate function Config
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAFMode == GPIO_MODE_AF){
        uint8_t altFunMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinAFMode;
        uint8_t temp1 = pinNumber/8;
        uint8_t temp2 = pinNumber%8;
        pGPIOHandle->pGPIOx->AFR[temp1] &=      ~(0xF << 4*temp2); 
        pGPIOHandle->pGPIOx->AFR[temp1] |= altFunMode << 4*temp2; 

        /*
        Eg: if pinNo = 9,
            9/8 = 1 (temp1)
            9%8 = 1 (temp2)
            AFR[temp1] => AFR[1] (AFRH)
            (4*temp2)'th Register => 4*1= 4th register (AFRH9)

        Eg: if pinNo = 4,
            4/8 = 0 (temp1)
            4%8 = 4 (temp2)
            AFR[temp1] => AFR[0] (AFRL)
            (4*temp2)'th Register => 4*4= 16th register (AFRL4)

        Ref 8.4.9 (Ref Manual)
        */
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
    if(pGPIOx==GPIOA)
        GPIOA_REG_RESET();
    if(pGPIOx==GPIOB)
        GPIOB_REG_RESET();
    if(pGPIOx==GPIOC)
        GPIOC_REG_RESET();
    if(pGPIOx==GPIOD)
        GPIOD_REG_RESET();
    if(pGPIOx==GPIOE)
        GPIOE_REG_RESET();
    if(pGPIOx==GPIOH)
        GPIOH_REG_RESET();
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
    uint16_t portValue;
    portValue = (uint16_t)(pGPIOx->IDR);
    return portValue;
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
    uint8_t pinValue;
    pinValue = (uint8_t)(GPIO_ReadFromInputPort(pGPIOx) >> pinNumber) & 0x1;
    return pinValue;
    /*
    Explanation w/ example:
        if pinNo = 8,    IDR = 0b1110100111100011
                                        ^ 8th pin
        IDR >> pinNo: IDR>>8 = 0b0000000011101001
        IDR >> pinNo & 0x1: 0b0000000011101001 & 0b000000000000001 = 1
        Hence, pinValue = 1
    */
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value){
    if(Value == SET)
        pGPIOx->ODR |= (1<<pinNumber);
    else
        pGPIOx->ODR &= ~(1<<pinNumber);
}



void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
    pGPIOx->ODR = Value;
}



void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
    pGPIOx->ODR ^= (1<<pinNumber);
}

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t Enable){
    uint8_t idx    = IRQNumber / 32;
    uint8_t offset = IRQNumber % 32;
    if(Enable)
        *NVIC_ISER(idx) |= (1<<offset);
    else
        *NVIC_ICER(idx) |= (1<<offset);
    /*
        Example w/ explanation:
        let our desired IRQ number to be enabled is 40 (EXTI15_10)
        so idx = 40/32 = 1
        &  offset = 40%32 = 8
        so we'll be setting 8th bit in NVIC_ISER1 register. 
    */
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
    uint8_t IPRx = IRQPriority / 4;
    uint8_t IPRx_PRIx = (IRQPriority % 4) * 0x08;
    *(NVIC_IPR + (IPRx * 0x04)) |= (IRQNumber << (IPRx_PRIx + 0x04));
    /*
        Explanation:
        IPRx -> is the priority register at offset x, holding 4 priority spaces of 8 bit each
        IPRx_PRIx -> offset of the priority(PRIx) section from base address of IPRx Reg.,
                        '0x08' is used, as each sections are of 8 bit length, 
                        and to calc offset from base address in multiple of 8

        (NVIC_IPR + (IPRx * 0x04)): IPRx is multiplied by 0x04, as address each register are at 4byte
                        offsets. Do note that IPRx is the register number, not its address. Code is 
                        written this way to improve readability.
        
        (IRQNumber << (IPRx_PRIx + 0x04)): PRIx is again offset by 0x04, as in ST MCUs, 
                        as of 8 bits, only 4 bits of interrupt priority are used (Ref Manual 10.1.1) 
    */
}

void GPIO_ISRHandling(uint8_t PinNumber){
    if(EXTI->PR & (1<<PinNumber))
        EXTI->PR |= (1<<PinNumber);
    /*
        This api is to be called mandatorily at start of every ISR, so as to avoid getting the IRQ
        raised over and over, due to errors like key bouncing.
    */
}
