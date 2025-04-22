/*
 * delay.c
 *
 *  Created on: Apr 22, 2025
 *      Author: nhh2k
 */

#include "stm32f4xx.h"
#include "delay.h"

void Timer1_Init(void) {
    // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
   
}

void delay_us(uint32_t us) {
     // Configure TIM1
    TIM1->PSC = 16-1;    // 72MHz/72 = 1MHz (1us resolution)
    TIM1->CR1 = 0x0000;  // Reset CR1 register
    TIM1->ARR = us;      // Set auto-reload value
    TIM1->EGR = TIM_EGR_UG;  // Generate update event
    TIM1->SR &= ~TIM_SR_UIF; // Clear update flag
    TIM1->CR1 |= TIM_CR1_CEN;  // Start counter
    
    // Wait until update flag is set
    while(!(TIM1->SR & TIM_SR_UIF));
    
    TIM1->SR &= ~TIM_SR_UIF;  // Clear update flag
    TIM1->CR1 &= ~TIM_CR1_CEN;  // Stop counter
}

// Optional: Millisecond delay function
void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}


