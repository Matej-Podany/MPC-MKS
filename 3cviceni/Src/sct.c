/*
 * sct.c
 *
 *  Created on: 12. 10. 2022
 *      Author: xpodan00
 */

#include "sct.h" // including header file

void sct_init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // ports B (PBx) will have clock enabled

	GPIOB->MODER |= GPIO_MODER_MODER3_0; // PB3 is initialized as output
	GPIOB->MODER |= GPIO_MODER_MODER4_0; // PB4 is initialized as output
	GPIOB->MODER |= GPIO_MODER_MODER5_0; // PB5 is initialized as output
	GPIOB->MODER |= GPIO_MODER_MODER10_0; // PB10 is initialized as output

	sct_led(0); // calling function sct_led() with argument (value) 0
	sct_noe(0); // calling macro sct_noe() with argument 0
}

void sct_led(uint32_t value) {
	for (uint8_t i = 0; i < 32; i++) {
		sct_sdi(value & 1); // returns the least significant bit LSB of value
		// and it is used as argument for calling sdi macro

		/*Generates one CLK pulse*/
		sct_clk(1);
		sct_clk(0);

		value >>= 1; // bit shift to the right (ex.: value before 0b00001011
		// value after 0b00000101
	}
	/*Generates one pulse on /LA */
	sct_nla(1);
	sct_nla(0);
}
