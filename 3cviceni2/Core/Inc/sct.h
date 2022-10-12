/*
 * sct.h
 *
 *  Created on: 12. 10. 2022
 *      Author: xpodan00
 */

#ifndef SCT_H_
#define SCT_H_

#include <stdint.h>
#include <stm32f0xx.h>

void sct_init(void); // function declaration

void sct_led(uint32_t value); // function declaration

void sct_value(uint16_t value); // function declaration

#endif /* SCT_H_ */
