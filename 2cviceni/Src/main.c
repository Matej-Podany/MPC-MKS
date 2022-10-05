/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f0xx.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_TIME_BLINK 300 // preprocesor replaces LED_TIME_BLINK with number 300

volatile uint32_t Tick; // global variable

void blikac(void)// this function is non-blockative counter for LED_TIME_BLINK ms
{
	static uint32_t delay;

	if (Tick > delay + LED_TIME_BLINK) {
		GPIOA->ODR ^= (1<<4); // reads value of LED1  and toggle LED1s value
		delay = Tick;
	}
}

int main(void)
{
	/*Setting clock pins*/
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; // enable clock GPIOA,B,C
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // SYSCFG clock enable

    /*Setting GPIO pins*/
    GPIOA->MODER |= GPIO_MODER_MODER4_0; // LED1 = PA4, output
    GPIOB->MODER |= GPIO_MODER_MODER0_0; // LED2 = PB0, output
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_0; // S2 = PC0, pullup
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_0; // S1 = PC1, pullup

    /*Definitions for interrupt*/
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC; // select PC0 for EXTI0
    EXTI->IMR |= EXTI_IMR_MR0; // mask
    EXTI->FTSR |= EXTI_FTSR_TR0; // trigger on falling edge
    NVIC_EnableIRQ(EXTI0_1_IRQn); // enable EXTI0_1

    /*Turning shields LED1 and LED2 on*/
    GPIOA->BSRR = (1<<4); // set LED1
    GPIOB->BSRR = (1<<0); // set LED2

    /*Setting tick frequency 8MHz core*/
    SysTick_Config(8000); // 1ms

    while (1) {
    	blikac(); // calling function for non-blockative counter
    }
}
void EXTI0_1_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) { // check line 0 has triggered the IT
    	EXTI->PR |= EXTI_PR_PR0; // clear the pending bit
    	GPIOB->ODR ^= (1<<0); // reads value of LED2  and toggle LED2s value

    }
}
void SysTick_Handler(void){
	Tick++; // important handler
}

