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

#define sct_nla(x) do { if (x) GPIOB->BSRR = (1 << 5); else GPIOB->BRR = (1 << 5); } while (0) // definition of macro for PB5
#define sct_sdi(x) do { if (x) GPIOB->BSRR = (1 << 4); else GPIOB->BRR = (1 << 4); } while (0) // definition of macro for PB4
#define sct_clk(x) do { if (x) GPIOB->BSRR = (1 << 3); else GPIOB->BRR = (1 << 3); } while (0) // definition of macro for PB3
#define sct_noe(x) do { if (x) GPIOB->BSRR = (1 << 10); else GPIOB->BRR = (1 << 10); } while (0) // definition of macro for PB10

void sct_init(void); // function declaration

void sct_led(uint32_t value); // function declaration

void sct_value(uint16_t value); // function declaration

#endif /* SCT_H_ */
