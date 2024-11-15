/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "init.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void configure_PC9_as_MCO(void)
{
	volatile uint32_t * RCC_AHB1ENR  = (volatile uint32_t*) RCC_AHB1ENR_ADD;
	volatile uint32_t * RCC_APB2ENR  = (volatile uint32_t *) RCC_APB2ENR_ADD;
	volatile uint32_t * RCC_CFGR     = (volatile uint32_t *) RCC_CFGR_ADD;
	volatile uint32_t * GPIOC_MODER  = (volatile uint32_t *) GPIOC_MODER_ADD;
	volatile uint32_t * GPIOC_AFRH   = (volatile uint32_t *) GPIOC_AFRH_ADD;

	*RCC_AHB1ENR |= (1<<2); //IO port C clock enable
	*RCC_APB2ENR |= (1<<14); //SYSCFGEN  System configuration controller clock enable set to 1
	*RCC_CFGR    &= ~(3<<30); //Clear MCO2 config Enable SYSCLK


	*GPIOC_MODER &= ~(3 << 18);//clear PC9 mode
	*GPIOC_MODER |=  (2 << 18); //PC9 is alternate
    *GPIOC_AFRH  &= ~(15<<  4); //AF0 (MCO)



}

void select_HSI_as_SYSCLK(void)
{
	volatile uint32_t * RCC_CR  = (volatile uint32_t*) RCC_CR_ADD;

	volatile uint32_t * RCC_CFGR     = (volatile uint32_t *) RCC_CFGR_ADD;

	*RCC_CR |= (0x01 << 0); //HSI ON
	*RCC_CFGR &= ~(0x03<<0); //Clear register and select HSI oscillator selected as system clock



	while(((*RCC_CFGR >>2) & 3) != 0)
	{
	}


}


int main(void)
{
    configure_PC9_as_MCO();
    select_HSI_as_SYSCLK();
}
