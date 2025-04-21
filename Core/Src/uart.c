// Include the appropriate header file for your STM32F4 series microcontroller
#include <uart.h>
void UART2_Init(void)
{
    // Enable clock for UART2 and GPIOA (PA2 and PA3)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable UART2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Enable GPIOA clock

    // Configure PA2 as UART2 TX (Alternate function mode)
    GPIOA->MODER |= GPIO_MODER_MODE2_1;      // PA2: Alternate function
    GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos); // AF7 (UART2) for PA2

    // Configure PA3 as UART2 RX (Alternate function mode)
    GPIOA->MODER |= GPIO_MODER_MODE3_1;      // PA3: Alternate function
    GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL3_Pos); // AF7 (UART2) for PA3

    // Configure UART2 settings: Baud Rate = 9600, 8 data bits, 1 stop bit, no parity
    USART2->BRR = 0x0683;   // Baud rate = 9600 (assuming 16 MHz PCLK1)
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable UART2 transmitter and UART2
}

void UART2_SendChar(char ch)
{
    // Wait until TXE (Transmit Data Register Empty) is set
    while (!(USART2->SR & USART_SR_TXE));

    // Send the character
    USART2->DR = ch;
}
void UART2_SendString(char* st ){
	int i = 0;
	while (*(st+i)){
		UART2_SendChar(st[i]);
		i++;
	}
}