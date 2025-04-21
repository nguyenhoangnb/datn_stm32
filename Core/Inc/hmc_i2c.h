/*
 * hmc_i2c.h
 *
 *  Created on: Apr 21, 2025
 *      Author: nhh2k
 */

#ifndef INC_HMC_I2C_H_
#define INC_HMC_I2C_H_
#include "stm32f4xx.h"
#include <uart.h>
#include <stdio.h>
#include <math.h>
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
uint8_t I2C_WaitForFlag(volatile uint32_t *reg, uint32_t flag, uint32_t timeout);
void I2C1_Init(void);
void I2C1_Write(uint8_t address, uint8_t reg, uint8_t data);
uint8_t I2C1_Read(uint8_t address, uint8_t reg);
void I2C1_ReadBurst(uint8_t address, uint8_t start_reg, uint8_t* buffer, uint8_t length);
uint8_t isDataReady(void);
#endif /* INC_HMC_I2C_H_ */
