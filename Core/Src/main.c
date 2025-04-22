#include "stm32f4xx.h"
#include<math.h>
#include "uart.h"
#include "hmc_i2c.h"
#include "hmc5883l.h"
#include<delay.h>
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
int main(void) {
    char buffer[50];
    HMC5883L_t hmc;
    float heading;

    // Initialize peripherals
    UART2_Init();  // Changed from UART2_Init
    UART2_SendString("HMC5883L Test\r\n");  // Changed from UART2_SendString

    // Initialize HMC5883L
    HMC5883L_Init(&hmc);

    // Perform calibration
    HMC5883L_Calibrate(&hmc);

    while(1) {
        // Get filtered and calibrated heading
        heading = HMC5883L_GetHeading(&hmc);

        // Send data via UART
        sprintf(buffer, "Heading: %.2f\r\n", heading);
        UART2_SendString(buffer);  // Changed from UART2_SendString

        // Small delay
        delay_ms(300);
    }

}

