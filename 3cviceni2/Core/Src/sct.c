/*
 * sct.c
 *
 *  Created on: 12. 10. 2022
 *      Author: xpodan00
 */

#include "main.h" // including header file
#include "sct.h" // including header file

void sct_init(void) {
	sct_led(0); // calling function sct_led() with argument (value) 0
}

void sct_led(uint32_t value) {
	for (uint8_t i = 0; i < 32; i++) {
		HAL_GPIO_WritePin(SCT_SDI_GPIO_Port, SCT_SDI_Pin, value & 1);
		// returns the least significant bit LSB of value
		// and it is set on SCT_SDI pin

		/*Generates one CLK pulse*/
		HAL_GPIO_WritePin(SCT_CLK_GPIO_Port, SCT_CLK_Pin, 1);
		HAL_GPIO_WritePin(SCT_CLK_GPIO_Port, SCT_CLK_Pin, 0);

		value >>= 1; // bit shift to the right (ex.: value before 0b00001011
		// value after 0b00000101
	}
	/*Generates one pulse on /LA */
	HAL_GPIO_WritePin(SCT_NLA_GPIO_Port, SCT_NLA_Pin, 1);
	HAL_GPIO_WritePin(SCT_NLA_GPIO_Port, SCT_NLA_Pin, 0);
}

void sct_value(uint16_t value) {
	uint32_t reg = 0;
	static const uint32_t reg_values[3][10] = {
			{
					//PCDE--------GFAB @ DIS1
					0b0111000000000111 << 16,
					0b0100000000000001 << 16,
					0b0011000000001011 << 16,
					0b0110000000001011 << 16,
					0b0100000000001101 << 16,
					0b0110000000001110 << 16,
					0b0111000000001110 << 16,
					0b0100000000000011 << 16,
					0b0111000000001111 << 16,
					0b0110000000001111 << 16,
			},
			{
					//----PCDEGFAB---- @ DIS2
					0b0000011101110000 << 0,
					0b0000010000010000 << 0,
					0b0000001110110000 << 0,
					0b0000011010110000 << 0,
					0b0000010011010000 << 0,
					0b0000011011100000 << 0,
					0b0000011111100000 << 0,
					0b0000010000110000 << 0,
					0b0000011111110000 << 0,
					0b0000011011110000 << 0,
			},
			{
					//PCDE--------GFAB @ DIS3
					0b0111000000000111 << 0,
					0b0100000000000001 << 0,
					0b0011000000001011 << 0,
					0b0110000000001011 << 0,
					0b0100000000001101 << 0,
					0b0110000000001110 << 0,
					0b0111000000001110 << 0,
					0b0100000000000011 << 0,
					0b0111000000001111 << 0,
					0b0110000000001111 << 0,
			},
	};
	reg |= reg_values[0][value / 100 % 10]; // set DIS3 number most right
	reg |= reg_values[1][value / 10 % 10]; // set DIS2 number
	reg |= reg_values[2][value / 1 % 10]; // set DIS1 number most left
	sct_led(reg); // sends the value to the display - reg includes all useful numbers and 8 bits are unused
}
