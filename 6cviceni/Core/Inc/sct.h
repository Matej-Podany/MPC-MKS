/*
 * sct.h
 *
 *  Created on: 12. 10. 2022
 *      Author: xpodan00
 */

#ifndef SCT_H_
#define SCT_H_


void sct_init(void); // function declaration

void sct_led(uint32_t value); // function declaration

void sct_value(uint16_t value, uint8_t led); // function declaration
// led means how many led's on bargraph will turn on

#endif /* SCT_H_ */
