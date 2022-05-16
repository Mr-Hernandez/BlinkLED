/*
 * IRQINTRO.c
 *
 *  Created on: May 14, 2022
 *      Author: medad
 */



#include <stdint.h>
#include "stm32f411.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
	// GPIOD seetings
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PClockControl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed);

    // GPIOA settings
    GPIO_Handle_t GpioButton;
    GpioButton.pGPIOx = GPIOD;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

    GPIO_PClockControl(GPIOD, ENABLE);
    GPIO_Init(&GpioButton);

    //IRQ Config
    GPIO_IRQPriorityConfig(IRQ_EXTI0, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_EXTI0, ENABLE);

//    while(1)
//    {
//    	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
//    	//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
//    	delay();
//    }

    while(1);


    return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
 	GPIO_IRQHandling(GPIO_PIN_NO_0);
	//GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
