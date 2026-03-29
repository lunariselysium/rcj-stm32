/**
 * @file witmotion_hwt101.c
 * @brief Implementation of HWT101 Driver
 */

#include "witmotion_hwt101.h"

/**
 * @brief Initialize the WitMotion Sensor Driver
 * @param dev Pointer to the driver handle
 * @param hi2c Pointer to the HAL I2C handle
 * @param addr_7bit The 7-bit I2C address (usually 0x50)
 */
HAL_StatusTypeDef WitMotion_Init(WitMotion_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit) {
    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    dev->hi2c = hi2c;
    dev->address = (addr_7bit << 1); // Shift for HAL 8-bit address format

    // Create a mutex for thread safety
    dev->lock = xSemaphoreCreateMutex();
    if (dev->lock == NULL) {
        return HAL_ERROR; // Failed to create mutex
    }

    // Test connection
    HAL_StatusTypeDef status;
    status = HAL_I2C_IsDeviceReady(dev->hi2c, 80 << 1, 5, 50);
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->address, 5, 50) != HAL_OK) {
        return HAL_ERROR; // Device not found
    }

    return HAL_OK;
}

/**
 * @brief Read data from the sensor and update internal state
 * @note  Reads Acceleration, Gyro, and Angle in one burst read.
 */
HAL_StatusTypeDef WitMotion_Update(WitMotion_Handle_t *dev) {
    if (dev == NULL) return HAL_ERROR;

    uint8_t raw_data[2];

    // Thread Safety: Take the Mutex
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return HAL_BUSY; // Could not acquire I2C bus lock
    }


    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c,
    		dev->address,
			WIT_REG_ANG_YAW,
			I2C_MEMADD_SIZE_8BIT,
			raw_data,
			2,
			100);

    // Release the Mutex immediately after I2C work is done
    xSemaphoreGive(dev->lock);

    if (status != HAL_OK) {
        return status;
    }

    dev->yaw = (int16_t)((raw_data[1] << 8) | raw_data[0]);

    return HAL_OK;
}


void WitMotion_TaskEntry(void *argument)
{
    // 1. Recover the handle from the void pointer
    WitMotion_Handle_t *dev = (WitMotion_Handle_t *)argument;

    if (dev == NULL || dev->hi2c == NULL) {
        // Error: Task started without a valid handle
        vTaskDelete(NULL);
    }

    // 2. Infinite Loop
    for(;;)
    {
        // Update the sensor data (internal mutex handles the I2C protection)
        if (WitMotion_Update(dev) != HAL_OK) {

        }

        // 3. Control Sampling Rate
        // 10ms = 100Hz
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief  Thread-safe getter for the raw sensor value.
 */
int16_t WitMotion_GetRawData(WitMotion_Handle_t *dev) {
    int16_t raw_yaw = 0;

    if (dev == NULL || dev->lock == NULL) {
        return 0; // Not initialized
    }

    // Protect reading the variable so we don't get half-updated data
    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        raw_yaw = dev->yaw;
        xSemaphoreGive(dev->lock);
    }

    return raw_yaw;
}

/**
 * @brief  Thread-safe getter for the latest yaw data in degrees.
 */
float WitMotion_GetYaw_Degrees(WitMotion_Handle_t *dev) {
    // Get the raw data using our thread-safe getter
    int16_t raw_value = WitMotion_GetRawData(dev);

    // Apply the conversion factor and return
    return (float)raw_value * WITMOTION_RAW_TO_DEG;
}
