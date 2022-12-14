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

int main(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
    GPIOA->BRR = (1<<5); // reset
    /*uint8_t pole[32] = {1,0,1,0,1,0,0,1,1,1,0,1,1,1,0,1,1,1,
    					0,0,1,0,1,0,1,0,0,0,0,0,0,0};
    // pole filled with sos sequence
	 Loop forever
	while (1) {
		for (uint8_t i = 0; i < 32; i++) { // loop for blinking
			if (pole[i] == 1){ // condition for SOS sequence
				GPIOA->BSRR = (1<<5); // set
			}
			else {
				GPIOA->BRR = (1<<5); // reset
			}
			for (volatile uint32_t i = 0; i < 100000; i++) {}
		}

	}*/
    uint32_t morse = 0b10101001110111011100101010000000;
    uint8_t i = 0;
    //testovat nejvyssi bit
    //(1UL << 31) je to same jako 0b10000000000000000000000000000000
    //(morse & 1UL << 31)
    //morse = morse << 1;
    while (1) {
    	if (morse & (1UL << 31)){
    		GPIOA->BSRR = (1<<5); // set
    	}
    	else {
    		GPIOA->BRR = (1<<5); // reset
    	}
    	morse = morse << 1;
    	if (i++ == 31){
    		morse = 0b10101001110111011100101010000000;
    		i = 0;
    	}
    	for (volatile uint32_t i = 0; i < 100000; i++) {}
    }
}
