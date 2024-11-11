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

#define RCC_BASE_ADD (uint32_t) 0x40023800
#define RCC_AHB1ENR_OFFSET  (uint32_t) 0x30
#define RCC_AHB1ENR_ADD  (RCC_BASE_ADD + RCC_AHB1ENR_OFFSET)

#define GPIOA_BASE_ADD     0x40020000
#define GPIOA_MODER_OFFSET       0x00
#define GPIOA_MODER_ADD  (GPIOA_BASE_ADD + GPIOA_MODER_OFFSET)
#define GPIOA_ODR_OFFSET         0x14
#define GPIOA_ODR_ADD    (GPIOA_BASE_ADD + GPIOA_ODR_OFFSET)


void delay (volatile unsigned int time);



volatile uint32_t * RCC_AHB1ENR = (uint32_t*) RCC_AHB1ENR_ADD;
volatile uint32_t * GPIOA_MODER = (uint32_t*) GPIOA_MODER_ADD;
// #define RCC_AHB1ENR  (*(uint32_t*))(RCC_BASE_ADD + RCC_AHB1ENR_OFFSET)
volatile  uint32_t * GPIOA_ODR =  (uint32_t*) GPIOA_ODR_ADD;

int main(void)
{
	*RCC_AHB1ENR |= 0x01;
	//clean register before set
	*GPIOA_MODER &= ~(3 << 10); // 0xffff f3ff
	*GPIOA_MODER |= (1<< 10);  //PA5 is output (01)


    /* Loop forever */
	while (1)
	{
		*GPIOA_ODR ^= (1<<5);
		delay(100000);

	}
}

void delay (volatile unsigned int time)
{
	while (time--)
	{
	}
}
