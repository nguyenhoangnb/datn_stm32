/*
 * hmc5883l.h
 *
 *  Created on: Apr 21, 2025
 *      Author: nhh2k
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32f4xx.h"
#include "filters.h"
#include "hmc_i2c.h"
#include<delay.h>

// Device and Register Addresses
#define HMC5883L_ADDR      0x1E
#define CONFIG_A           0x00
#define CONFIG_B           0x01
#define MODE_REG           0x02
#define X_MSB              0x03
#define Z_MSB              0x05
#define Y_MSB              0x07
#define STATUS_REG         0x09 

// Status bits
#define STATUS_RDY_BIT     0x01
#define STATUS_LOCK_BIT    0x02

// Error codes
#define HMC_ERR_NONE       0x00
#define HMC_ERR_LOCK       0x01
#define HMC_ERR_TIMEOUT    0x02

// Configuration
#define PI                 3.14159f
#define DECLINATION_ANGLE  0.0f
#define SAMPLE_COUNT       300

typedef struct {
    uint8_t error_code;
    uint8_t retry_count;
    uint8_t last_status;
} HMC5883L_Status;

typedef struct {
    float m_bias[3];
    float m_scale[3];
    MovingAverageFilter avg_filter;
    MedianFilter med_filter;
    float current_heading;
} HMC5883L_t;

void HMC5883L_Init(HMC5883L_t* hmc);
uint8_t HMC5883L_Calibrate(HMC5883L_t* hmc);
float HMC5883L_GetHeading(HMC5883L_t* hmc);
uint8_t HMC5883L_Reset(HMC5883L_t* hmc);
uint8_t HMC5883L_IsDataLocked(void);

#endif
