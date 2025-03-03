/*
 * gpio_button_interrupt.c
 *
 *
 *      Author: Chetan
 */


#include <stdint.h>
#include <string.h>
#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"

int main(void)
{
  GPIO_Handle_t LEDGpio;
  LEDGpio.pGPIOx = GPIOB;
  LEDGpio.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
  LEDGpio.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_7;
  LEDGpio.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
  LEDGpio.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PULL_UP;
  LEDGpio.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SPEED_HI;

  GPIO_PClockControl(GPIOB, ENABLE);
  GPIO_Init(&LEDGpio);

  GPIO_Handle_t builtinLEDGpio;
  builtinLEDGpio.pGPIOx = GPIOC;
  builtinLEDGpio.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
  builtinLEDGpio.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_13;
  builtinLEDGpio.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
  builtinLEDGpio.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
  builtinLEDGpio.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SPEED_HI;

  GPIO_PClockControl(GPIOC, ENABLE);
  GPIO_Init(&builtinLEDGpio);

  GPIO_Handle_t buttonGPIO;
  memset(&buttonGPIO, 0, sizeof(buttonGPIO));
  buttonGPIO.pGPIOx = GPIOA;
  buttonGPIO.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
  buttonGPIO.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_0;
  buttonGPIO.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
  buttonGPIO.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
  buttonGPIO.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OP_SPEED_LOW;

  GPIO_PClockControl(GPIOA, ENABLE);
  GPIO_Init(&buttonGPIO);

//  Following IRQ Handler responds to a button press which toggles an LED attached to a GPIO pin
  GPIO_IRQITConfig(IRQ_EXTI0, ENABLE);

  /* Loop forever */
	for(;;){
		// Following loop toggles the builtin led on the board
		GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_13);
		for(uint32_t i=0; i<200000; i++);
  }
}

void EXTI0_IRQHandler(void){
  for(uint32_t i=0; i<600000; i++);
  GPIO_ISRHandling(GPIO_PIN_0);
  GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_7, RESET);
  GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_7);
}
