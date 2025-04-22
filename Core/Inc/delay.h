/*
 * delay.h
 *
 *  Created on: Apr 22, 2025
 *      Author: nhh2k
 */

#ifndef DELAY_H
#define DELAY_H

#include "stm32f4xx.h"

void Timer1_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* INC_DELAY_H_ */
