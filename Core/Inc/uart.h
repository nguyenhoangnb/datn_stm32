#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdio.h>

// UART Configuration structure
void UART_Init();
void UART_SendChar(char);
void UART_SendString(char*);

#endif