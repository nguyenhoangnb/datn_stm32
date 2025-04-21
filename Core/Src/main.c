#include "stm32f4xx.h"
#include<uart.h>
#include<hmc_i2c.h>
#define HMC5883L_ADDR 0x1E
#define CONFIG_A 0x00
#define CONFIG_B 0x01
#define MODE_REG 0x02
#define X_MSB 0x03
#define Z_MSB 0x05
#define Y_MSB 0x07
#define STATUS_REG 0x09
#define DATA_READY_MASK 0x01
#define STATUS_LOCK_MASK 0x02
#define STATUS_READY_MASK 0x01

// Add timeout mechanism
#define I2C_TIMEOUT 10000

// Add this function to check for timeout
// uint8_t I2C_WaitForFlag(volatile uint32_t *reg, uint32_t flag, uint32_t timeout)
// {
//     uint32_t time = timeout;
//     while((!(*reg & flag)) && (time > 0)) {
//         time--;
//     }
//     return (time > 0);
// }

// void UART2_Init(void)
// {
//     // Enable clock for UART2 and GPIOA (PA2 and PA3)
//     RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable UART2 clock
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Enable GPIOA clock

//     // Configure PA2 as UART2 TX (Alternate function mode)
//     GPIOA->MODER |= GPIO_MODER_MODE2_1;      // PA2: Alternate function
//     GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos); // AF7 (UART2) for PA2

//     // Configure PA3 as UART2 RX (Alternate function mode)
//     GPIOA->MODER |= GPIO_MODER_MODE3_1;      // PA3: Alternate function
//     GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL3_Pos); // AF7 (UART2) for PA3

//     // Configure UART2 settings: Baud Rate = 9600, 8 data bits, 1 stop bit, no parity
//     USART2->BRR = 0x0683;   // Baud rate = 9600 (assuming 16 MHz PCLK1)
//     USART2->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable UART2 transmitter and UART2
// }

// void UART2_SendChar(char ch)
// {
//     // Wait until TXE (Transmit Data Register Empty) is set
//     while (!(USART2->SR & USART_SR_TXE));

//     // Send the character
//     USART2->DR = ch;
// }
// void UART2_SendString(char* st ){
// 	int i = 0;
// 	while (*(st+i)){
// 		UART2_SendChar(st[i]);
// 		i++;
// 	}
// }

// // Modify I2C1_Init function
// void I2C1_Init(void)
// {
//     // Reset I2C first
//     RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
//     for(volatile int i = 0; i < 100; i++);
//     RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

// 	//Clock
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//     RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

// 	//Analog mode
//     GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;

//     //Open drain
// 	GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;

//     //High speed output
// 	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;

//     //Pull up resistor
// 	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;

//     //Alternative function for i2c
// 	GPIOB->AFR[1] |= (4 << 0) | (4 << 4);

//     // Make sure the bus is free
//     while(I2C1->SR2 & I2C_SR2_BUSY) {
//         // Toggle I2C pins to release bus
//         GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
//         for(volatile int i = 0; i < 100; i++);
//         GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
//     }

// 	//Reset I2C
// 	I2C1->CR1 |= (1<<15);
// 	I2C1->CR1 &= ~(1<<15);

// 	//I2C clock bus
//     I2C1->CR2 |= (42<<0);
//     I2C1->CCR = 210<<0;
//     I2C1->TRISE = 43;

//     //I2C enable
// 	I2C1->CR1 |= I2C_CR1_PE;
// }

// // Modify I2C1_Write function
// void I2C1_Write(uint8_t address, uint8_t reg, uint8_t data)
// {
//     // Start condition
//     I2C1->CR1 |= I2C_CR1_ACK;
//     I2C1->CR1 |= I2C_CR1_START;
//     while(!(I2C1->SR1 & I2C_SR1_SB));

//     // Send slave address (write mode)
//     I2C1->DR = address << 1;  // Shift left 1 bit for write mode (0)
//     while(!(I2C1->SR1 & I2C_SR1_ADDR));
//     (void)I2C1->SR1;
//     (void)I2C1->SR2;

//     // Send register address
//     while(!(I2C1->SR1 & I2C_SR1_TXE));
//     I2C1->DR = reg;
//     while(!(I2C1->SR1 & I2C_SR1_BTF));

//     // Send data
//     while(!(I2C1->SR1 & I2C_SR1_TXE));
//     I2C1->DR = data;
//     while(!(I2C1->SR1 & I2C_SR1_BTF));

//     // Stop condition
//     I2C1->CR1 |= I2C_CR1_STOP;
// }

// uint8_t I2C1_Read(uint8_t address, uint8_t reg)
// {
//     uint8_t data = 0;

//     // Start condition
//     I2C1->CR1 |= I2C_CR1_ACK;
//     I2C1->CR1 |= I2C_CR1_START;
//     while(!(I2C1->SR1 & I2C_SR1_SB));

//     // Send slave address (write mode)
//     I2C1->DR = address << 1;  // Shift left 1 bit for write mode (0)
//     while(!(I2C1->SR1 & I2C_SR1_ADDR));
//     (void)I2C1->SR1;
//     (void)I2C1->SR2;

//     // Send register address
//     while(!(I2C1->SR1 & I2C_SR1_TXE));
//     I2C1->DR = reg;
//     while(!(I2C1->SR1 & I2C_SR1_BTF));

//     // Restart for reading
//     I2C1->CR1 |= I2C_CR1_START;
//     while(!(I2C1->SR1 & I2C_SR1_SB));

//     // Send slave address (read mode)
//     I2C1->DR = (address << 1) | 0x1;  // Shift left 1 bit and set read bit (1)
//     while(!(I2C1->SR1 & I2C_SR1_ADDR));
    
//     // Clear ACK bit and set STOP
//     I2C1->CR1 &= ~I2C_CR1_ACK;
//     (void)I2C1->SR1;
//     (void)I2C1->SR2;

//     // Wait for data and read it
//     while(!(I2C1->SR1 & I2C_SR1_RXNE));
//     data = I2C1->DR;

//     // Stop condition
//     I2C1->CR1 |= I2C_CR1_STOP;

//     return data;
// }

// // Add this new function to read multiple registers in sequence
// void I2C1_ReadBurst(uint8_t address, uint8_t start_reg, uint8_t* buffer, uint8_t length)
// {
//     // Start condition
//     I2C1->CR1 |= I2C_CR1_ACK;
//     I2C1->CR1 |= I2C_CR1_START;
//     while(!(I2C1->SR1 & I2C_SR1_SB));

//     // Send slave address (write mode) to set register pointer
//     I2C1->DR = address << 1;
//     while(!(I2C1->SR1 & I2C_SR1_ADDR));
//     (void)I2C1->SR1;
//     (void)I2C1->SR2;

//     // Send start register address
//     while(!(I2C1->SR1 & I2C_SR1_TXE));
//     I2C1->DR = start_reg;
//     while(!(I2C1->SR1 & I2C_SR1_BTF));

//     // Repeated start
//     I2C1->CR1 |= I2C_CR1_START;
//     while(!(I2C1->SR1 & I2C_SR1_SB));

//     // Send slave address (read mode)
//     I2C1->DR = (address << 1) | 0x1;
//     while(!(I2C1->SR1 & I2C_SR1_ADDR));
//     (void)I2C1->SR1;
//     (void)I2C1->SR2;

//     // Read multiple bytes using auto-increment
//     for(uint8_t i = 0; i < length - 1; i++) {
//         // Keep ACK high to continue reading
//         I2C1->CR1 |= I2C_CR1_ACK;
//         while(!(I2C1->SR1 & I2C_SR1_RXNE));
//         buffer[i] = I2C1->DR;
//     }

//     // Read last byte with NACK
//     I2C1->CR1 &= ~I2C_CR1_ACK;
//     while(!(I2C1->SR1 & I2C_SR1_RXNE));
//     buffer[length-1] = I2C1->DR;

//     // Stop condition
//     I2C1->CR1 |= I2C_CR1_STOP;
// }

// // Modify the isDataReady function to include better checking and logging
// uint8_t isDataReady(void)
// {
//     uint8_t status = I2C1_Read(HMC5883L_ADDR, STATUS_REG);
//     char buffer[50];

//     // Check for data lock (overwrite prevented)
//     if (status & STATUS_LOCK_MASK) {
//         sprintf(buffer, "WARNING: Data lock detected (0x%02X)\r\n", status);
//         UART2_SendString(buffer);
//         return 0;
//     }

//     // Check for data ready
//     if (status & STATUS_READY_MASK) {
//         return 1;
//     }

//     // Optional: Log when waiting for too long
//     static uint32_t wait_count = 0;
//     wait_count++;
//     if (wait_count > 1000) {
//         sprintf(buffer, "INFO: Waiting for data... Status: 0x%02X\r\n", status);
//         UART2_SendString(buffer);
//         wait_count = 0;
//     }

//     return 0;
// }

// Modify main function
int main(void)
{
	
   UART2_Init();
   UART2_SendString("Hello\n");
 
   
   I2C1_Init();

   // Initialize HMC5883L
   I2C1_Write(HMC5883L_ADDR, CONFIG_A, 0x70);  // 8-average, 15 Hz
   I2C1_Write(HMC5883L_ADDR, CONFIG_B, 0x20);  // Gain=1090 LSB/Gauss
   I2C1_Write(HMC5883L_ADDR, MODE_REG, 0x00);  // Continuous measurement mode

   uint8_t data[6];
   int16_t x, y, z;
   char buffer[50];
   float heading;

   while (1)
   {
       // Wait for new data to be ready
       if(isDataReady())
       {
           // Read all 6 bytes starting from X_MSB using auto-increment
           I2C1_ReadBurst(HMC5883L_ADDR, X_MSB, data, 6);

           // Convert the data
           x = (int16_t)((data[0] << 8) | data[1]);  // X axis
           z = (int16_t)((data[2] << 8) | data[3]);  // Z axis
           y = (int16_t)((data[4] << 8) | data[5]);  // Y axis

           // Calculate heading (optional)
           heading = atan2f((float)y, (float)x) * 180.0f / 3.14159f;
           if(heading < 0) heading += 360.0f;

           // Format and send via UART
           sprintf(buffer, "X:%d,Y:%d,Z:%d,H:%.1f\r\n", x, y, z, heading);
           UART2_SendString(buffer);
       }

       // Small delay to prevent overwhelming the I2C bus
       for(volatile int i = 0; i < 100000; i++);
   }
}
