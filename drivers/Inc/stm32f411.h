/*
 * stm32f411.h
 *
 *  Created on: May 5, 2022
 *      Author: medad
 */

#ifndef INC_STM32F411_H_
#define INC_STM32F411_H_

#include <stdint.h>
#define SRAM_ADDR				0x20000000U
#define FLASH					0x08000000U
#define ROM						0x1FFF0000U  /* System Memory */

#define APB1_ADDR				0x40000000U
#define TIM2_ADDR				(APB1_ADDR)
#define TIM3_ADDR				((APB1_ADDR) + (0x0400))
#define TIM4_ADDR				((APB1_ADDR) + (0x0800))
#define TIM5_ADDR				((APB1_ADDR) + (0x0C00))
#define RTC_ADDR				((APB1_ADDR) + (0x2800))
#define WWDG_ADDR				((APB1_ADDR) + (0x2C00))
#define IWDG_ADDR				((APB1_ADDR) + (0x3000))
#define I2S2ext_ADDr			((APB1_ADDR) + (0x3400))
#define SPI2_ADDR				((APB1_ADDR) + (0x3800))
#define SPI3_ADDR				((APB1_ADDR) + (0x3C00))
#define I2S3ext_ADDR			((APB1_ADDR) + (0x4000))
#define USART2_ADDR				((APB1_ADDR) + (0x4400))
#define I2C1_ADDR				((APB1_ADDR) + (0x5400))
#define I2C2_ADDR				((APB1_ADDR) + (0x5800))
#define I2C3_ADDR				((APB1_ADDR) + (0x5C00))
#define PWR_ADDR				((APB1_ADDR) + (0x7000))

#define APB2_ADDR				0x40010000
#define TIM1_ADDR				((APB2_ADDR) + (0x0000))
#define USART1_ADDR				((APB2_ADDR) + (0x1000))
#define USART6_ADDR				((APB2_ADDR) + (0x1400))
#define ADC1_ADDR				((APB2_ADDR) + (0x2000))
#define SDIO_ADDR				((APB2_ADDR) + (0x2C00))
#define SPI1_ADDR				((APB2_ADDR) + (0x3000))
#define SPI4_ADDR				((APB2_ADDR) + (0x3400))
#define SYSCFG_ADDR				((APB2_ADDR) + (0x3800))
#define EXTI_ADDR				((APB2_ADDR) + (0x3C00))
#define TIM9_ADDR				((APB2_ADDR) + (0x4000))
#define TIM10_ADDR				((APB2_ADDR) + (0x4400))
#define TIM11_ADDR				((APB2_ADDR) + (0x4800))
#define SPI5_ADDR				((APB2_ADDR) + (0x5000))

#define AHB1_ADDR				0x40020000U
#define GPIOA_ADDR				((AHB1_ADDR) + (0x0000U))
#define GPIOB_ADDR				((AHB1_ADDR) + (0x0400U))
#define GPIOC_ADDR				((AHB1_ADDR) + (0x0800U))
#define GPIOD_ADDR				((AHB1_ADDR) + (0x0C00U))
#define GPIOE_ADDR				((AHB1_ADDR) + (0x1000U))
#define GPIOH_ADDR				((AHB1_ADDR) + (0x1C00U))
#define CRC_ADDR				((AHB1_ADDR) + (0x3000U))
#define RCC_ADDR				((AHB1_ADDR) + (0x3800U))
#define FLASH_INTERFACE_ADDR	((AHB1_ADDR) + (0x3C00U))
#define DMA1_ADDR				((AHB1_ADDR) + (0x6000U))
#define DMA2_ADDR				((AHB1_ADDR) + (0x6400U))

#define AHB2_ADDR				0x50000000U
#define USB_OTG_FS_ADDR			(AHB2_ADDR)

#define NVIC_ADDR				(0xE000E100U)
#define NVIC_STIR_ADDR			(0xE000EF00U)

#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)
#define NVIC_ICER0				((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0xE000E188)
#define NVIC_IPR_ADDR			((volatile uint32_t*)0xE000E400)


//#define NVIC_ISER1_ADDR			(0xE000E104U)
//#define NVIC_ICER0_ADDR			(0xE000E180U)
//#define NVIC_ICER1_ADDR			(0xE000E184U)
//#define NVIC_IPR0_ADDR			(0xE000E400U)
//#define NVIC_IPR1_ADDR			(0xE000E404U)


// GPIOx Register Definition
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} GPIO_RegDef_t;

// RCC Register Definition
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t RESERVED0;
	uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED6;
	uint32_t RESERVED7;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t RESERVED8;
	uint32_t RESERVED9;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED10;
	uint32_t RESERVED11;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED12;
	uint32_t RESERVED13;
	volatile uint32_t SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
	uint32_t RESERVED14;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;

typedef struct{

}I2C_RegDef_t;

typedef struct{

}SPI_RegDef_t;

typedef struct{

}USART_RegDef_t;


// EXTI Register Definition
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

// NVIC Register Definition
//typedef struct{
//	volatile uint32_t ISER[8];  // last register is only 16 bits (16-31 are reserved)
//	uint32_t RESERVED[6];
//	volatile uint32_t ICER[8];  // last register is only 16 bits (16-31 are reserved)
//	volatile uint32_t ISPR[8];
//	uint32_t RESERVED2[6];
//	volatile uint32_t ICPR[8];
//	volatile uint32_t IABR[8];
//	volatile uint32_t IPR[60];  // Each 32bit register is sectioned into 4 8-bit, represents priority
//	uint32_t RESERVED3[581];     // Starts at 0x4EC ends at 0xDFC (offsets) // prob get rid of or something
//	volatile uint32_t STIR;     // Starts at 0xE00 offset
//
//}NVIC_RegDef_t;

// SYSCFG Register Definition
typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	volatile uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 *  Peripheral definitions. ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_ADDR)

#define RCC						((RCC_RegDef_t*)RCC_ADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_ADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_ADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_ADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_ADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_ADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_ADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_ADDR)
#define SPI5					((SPI_RegDef_t*)SPI5_ADDR)

#define USART1					((USART_RegDef_t*)USART1_ADDR)
#define USART2					((USART_RegDef_t*)USART2_ADDR)
#define USART6					((USART_RegDef_t*)USART6_ADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_ADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_ADDR)

#define NVIC					((NVIC_RegDef_t*)NVIC_ADDR)



/*
 * Clock enable/disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7))

/*
 * GPIO Reset macros
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)


/*
 *  return port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOH) ? 5 : 0 )
/*
 * Clock enable/disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN			(RCC->APB1ENR |= (1 << 23))

#define I2C1_PCLK_DI			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI			(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock enable/disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN			(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN			(RCC->APB2ENR |= (1 << 20))

#define SPI1_PCLK_DI			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI			(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI			(RCC->APB2ENR &= ~(1 << 20))


/*
 * Clock enable/disable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN			(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN			(RCC->APB2ENR |= (1 << 5))

#define USART1_PCLK_DI			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI			(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI			(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock enable/disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))


/*
 * EXTI PERIPHERAL ADDRESSES
 */

/*
 * Interrupt Positions
 */
#define IRQ_EXTI0				6
#define IRQ_EXTI1				7
//... add more from vector table here

/*
 * Interrtup Priority
 */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI15			15

#define NO_OF_PRI_BITS_IMPLEMENTED	4

/*
 * Some Generic Macros
 */

#define ENABLE 					1
#define DISABLE					0
#define SET 					ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

#include "stm32f4xx_gpio_driver.h"



#endif /* INC_STM32F411_H_ */
