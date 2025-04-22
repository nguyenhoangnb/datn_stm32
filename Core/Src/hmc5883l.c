/*
 * hmc5883l.c
 *
 *  Created on: Apr 21, 2025
 *      Author: nhh2k
 */

#include "hmc5883l.h"
#include "uart.h"
#include <math.h>
#include "stm32f4xx_hal.h" // Keil::Device:STM32Cube HAL:Common
#define MAX_RETRIES 3
#define RESET_DELAY 1000

static HMC5883L_Status sensor_status = {HMC_ERR_NONE, 0, 0};

uint8_t HMC5883L_IsDataLocked(void) {
    uint8_t status = I2C1_Read(HMC5883L_ADDR, STATUS_REG);
    return (status & STATUS_LOCK_BIT) ? 1 : 0;
}

uint8_t HMC5883L_Reset(HMC5883L_t* hmc) {
    uint8_t retry = 0;
    uint8_t max_retry = 3;
    
    while(retry < max_retry) {
        // Power cycle by toggling mode
        I2C1_Write(HMC5883L_ADDR, MODE_REG, 0x02);  // Idle mode
        for(volatile uint32_t i = 0; i < RESET_DELAY; i++);
        I2C1_Write(HMC5883L_ADDR, MODE_REG, 0x00);  // Continuous mode
        
        // Wait for sensor to stabilize
        for(volatile uint32_t i = 0; i < RESET_DELAY * 2; i++);
        
        // Check if still locked
        uint8_t status = I2C1_Read(HMC5883L_ADDR, STATUS_REG);
        if(!(status & STATUS_LOCK_BIT)) {
            UART2_SendString("Reset successful\r\n");
            return 1;
        }
        
        retry++;
        UART2_SendString("Reset attempt failed, retrying...\r\n");
    }
    
    return 0;
}

void HMC5883L_Init(HMC5883L_t* hmc) {
    // Power-on reset delay
    Timer1_Init();
    I2C1_Init();  // Initialize I2C
    // Configure with delays between writes
    I2C1_Write(HMC5883L_ADDR, CONFIG_A, 0x70);  // 8-average, 15 Hz
    I2C1_Write(HMC5883L_ADDR, CONFIG_B, 0x20);  // Gain=1090 LSB/Gauss
    I2C1_Write(HMC5883L_ADDR, MODE_REG, 0x00);  // Continuous measurement
    
    // // Verify configuration
    // uint8_t config_a = I2C1_Read(HMC5883L_ADDR, CONFIG_A);
    // uint8_t config_b = I2C1_Read(HMC5883L_ADDR, CONFIG_B);
    // uint8_t mode = I2C1_Read(HMC5883L_ADDR, MODE_REG);
    
    // if(config_a != 0x70 || config_b != 0x20 || mode != 0x00) {
    //     UART2_SendString("Initialization failed!\r\n");
    //     // Handle error
    // }
    
    // Initialize calibration values
    for(int i = 0; i < 3; i++) {
        hmc->m_bias[i] = 0.0f;
        hmc->m_scale[i] = 1.0f;
    }
    
    // Initialize filters
    MovingAverage_Init(&hmc->avg_filter);
    MedianFilter_Init(&hmc->med_filter);
    
    hmc->current_heading = 0.0f;
    
    UART2_SendString("HMC5883L Initialized\r\n");
}

// Modify the Calibrate function to check for locked data
uint8_t HMC5883L_Calibrate(HMC5883L_t* hmc) {
    float mag_min[3] = {999999.0f, 999999.0f, 999999.0f};
    float mag_max[3] = {-999999.0f, -999999.0f, -999999.0f};
    uint8_t data[6];
    int16_t x, y, z;
    uint32_t start_time;
    char buffer[50];
    
    UART2_SendString("Starting 3-second calibration...\r\n");
    UART2_SendString("Rotate sensor in all directions!\r\n");
    
    // Delay before starting
    delay_ms(10);
    
    // Collect samples for 3 seconds
    for(start_time = 0; start_time < SAMPLE_COUNT; start_time++) {
        // uint8_t status = I2C1_Read(HMC5883L_ADDR, STATUS_REG);
        
        // Check for progress update
        // if(start_time >= next_progress) {
        //     sprintf(buffer, "Calibrating: %d%%\r\n", (start_time * 100) / calibration_time);
        //     UART2_SendString(buffer);
        //     next_progress += progress_interval;
        // }
        if(isDataReady()) {
            I2C1_ReadBurst(HMC5883L_ADDR, X_MSB, data, 6);
            
            x = (int16_t)((data[0] << 8) | data[1]);
            z = (int16_t)((data[2] << 8) | data[3]);
            y = (int16_t)((data[4] << 8) | data[5]);
            
            // Update min/max
            if((float)x < mag_min[0]) mag_min[0] = (float)x;
            if((float)x > mag_max[0]) mag_max[0] = (float)x;
            if((float)y < mag_min[1]) mag_min[1] = (float)y;
            if((float)y > mag_max[1]) mag_max[1] = (float)y;
            if((float)z < mag_min[2]) mag_min[2] = (float)z;
            if((float)z > mag_max[2]) mag_max[2] = (float)z;
            
            // Small delay between samples
        } else{
            UART2_SendString("Data locked during calibration! Restarting...\r\n");
            // return 0; // Indicate calibration failure
            continue;
        }
        delay_ms(100);  // Small delay between samples
    }
    
    // Calculate calibration parameters
    float avg_radius = 0.0f;
    float scale_factors[3];
    
    for(uint8_t i = 0; i < 3; i++) {
        hmc->m_bias[i] = (mag_max[i] + mag_min[i]) / 2.0f;
        scale_factors[i] = (mag_max[i] - mag_min[i]) / 2.0f;
        avg_radius += scale_factors[i];
    }
    
    avg_radius /= 3.0f;
    
    // Calculate final scale values
    for(uint8_t i = 0; i < 3; i++) {
        hmc->m_scale[i] = avg_radius / scale_factors[i];
        if(isnanf(hmc->m_scale[i])) {
            hmc->m_scale[i] = 1.0f;
        }
    }
    
    sprintf(buffer, "Calibration complete!\r\nBias: %.1f, %.1f, %.1f\r\n", 
            hmc->m_bias[0], hmc->m_bias[1], hmc->m_bias[2]);
    UART2_SendString(buffer);
    
    sprintf(buffer, "Scale: %.3f, %.3f, %.3f\r\n", 
            hmc->m_scale[0], hmc->m_scale[1], hmc->m_scale[2]);
    UART2_SendString(buffer);
    
    return 1;
}

// Modify the GetHeading function to check for locked data
float HMC5883L_GetHeading(HMC5883L_t* hmc) {
    uint8_t status = I2C1_Read(HMC5883L_ADDR, STATUS_REG);
    sensor_status.last_status = status;
    
    if(status & STATUS_LOCK_BIT) {
        char buffer[50];
        sprintf(buffer, "Lock detected! Status: 0x%02X\r\n", status);
        UART2_SendString(buffer);
        
        // Attempt reset if not exceeded max retries
        if(sensor_status.retry_count < MAX_RETRIES) {
            sensor_status.retry_count++;
            if(HMC5883L_Reset(hmc)) {
                UART2_SendString("Resuming operation...\r\n");
            } else {
                UART2_SendString("Reset failed! Check connections.\r\n");
            }
        } else {
            UART2_SendString("Max retries exceeded! Hardware error?\r\n");
            sensor_status.error_code = HMC_ERR_TIMEOUT;
        }
        return hmc->current_heading;  // Return last valid heading
    }
    
    // Reset retry count on successful read
    if(status & STATUS_RDY_BIT) {
        sensor_status.retry_count = 0;
        
        uint8_t data[6];
        float heading = 0.0f;
        
        I2C1_ReadBurst(HMC5883L_ADDR, X_MSB, data, 6);
        
        // Get raw values
        float x = (float)((int16_t)((data[0] << 8) | data[1]));
        float y = (float)((int16_t)((data[4] << 8) | data[5]));
        
        // Apply calibration
        x = (x - hmc->m_bias[0]) * hmc->m_scale[0];
        y = (y - hmc->m_bias[1]) * hmc->m_scale[1];
        
        // Calculate heading
        heading = atan2f(y, x);
        
        // Add declination
        heading += DECLINATION_ANGLE * PI / 180.0f;
        
        // Normalize to 0-2π
        if(heading < 0) {
            heading += 2 * PI;
        }
        
        // Convert to degrees
        heading = heading * 180.0f / PI;
        
        // Apply filters
        float heading_median = MedianFilter_Update(&hmc->med_filter, heading);
        float heading_avg = MovingAverage_Update(&hmc->avg_filter, heading_median);
        
        hmc->current_heading = heading_avg;
    }
    
    return hmc->current_heading;
}


