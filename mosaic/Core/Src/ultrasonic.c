#include "ultrasonic.h"

int16_t DYP_ReadDistance(uint8_t address){
    uint8_t cmd_data = CMD_TRIG_MODE4_MM;
    uint8_t rx_data[2];
    HAL_StatusTypeDef ret;

    // Send Trigger Command
    ret = HAL_I2C_Mem_Write(&hi2c2, address, REG_COMMAND, I2C_MEMADD_SIZE_8BIT, &cmd_data, 1, 100);
    if (ret != HAL_OK){
        return -1;
    }

    // Wait for measurement (Mode 4 requires processing time)
    // Note: If reading 4 sensors, consider triggering all 4 first,
    // waiting once, then reading all 4 to save time.
    HAL_Delay(150);

    // Read Distance Data
    ret = HAL_I2C_Mem_Read(&hi2c2, address, REG_DIST_HIGH, I2C_MEMADD_SIZE_8BIT, rx_data, 2, 100);
    if (ret != HAL_OK){
        return -1;
    }

    uint16_t distance = (rx_data[0]<<8) | rx_data[1];

    if (distance >= 0xFFF0){
        return -2; // Error or Out of Range
    }

    return (int16_t)distance;
}


#include "ultrasonic.h"

// 1. Send the command to start measurement
HAL_StatusTypeDef DYP_Trigger(uint8_t address){
    uint8_t cmd_data = CMD_TRIG_MODE4_MM;
    return HAL_I2C_Mem_Write(&hi2c2, address, REG_COMMAND, I2C_MEMADD_SIZE_8BIT, &cmd_data, 1, 100);
}

// 2. Read the data (After 150ms)
int16_t DYP_GetDistance(uint8_t address){
    uint8_t rx_data[2];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c2, address, REG_DIST_HIGH, I2C_MEMADD_SIZE_8BIT, rx_data, 2, 100);
    if (ret != HAL_OK){
        return -1; // I2C Error
    }

    uint16_t distance = (rx_data[0]<<8) | rx_data[1];

    if (distance >= 0xFFF0){
        return -2; // Out of Range or invalid
    }

    return (int16_t)distance;
}



HAL_StatusTypeDef DYP_SetAddress(uint8_t current_addr, uint8_t new_addr){
    // Write the new address to register 0x05
    // Note: new_addr must be the 8-bit address (e.g., 0xD2)
    return HAL_I2C_Mem_Write(&hi2c2, current_addr, REG_ADDR_CONFIG, I2C_MEMADD_SIZE_8BIT, &new_addr, 1, 100);
}
