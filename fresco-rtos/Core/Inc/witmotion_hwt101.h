/**
 * @file witmotion_hwt101.h
 * @brief FreeRTOS compatible I2C Driver for WitMotion HWT101 Sensor
 */

#ifndef WITMOTION_HWT101_H
#define WITMOTION_HWT101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define HWT101_I2C_ADDR_DEFAULT  (0x50)

/* Register Map (WitMotion Standard Protocol) */
#define WIT_REG_SAVE      0x00
#define WIT_REG_CALSW     0x01
#define WIT_REG_RSW       0x02
#define WIT_REG_RATE      0x03
#define WIT_REG_BAUD      0x04
#define WIT_SAVE_CMD      0x00
#define WIT_REG_ANG_YAW 0x3F

#define WITMOTION_RAW_TO_DEG (180.0f / 32768.0f)

/**
 * @brief Driver Handle
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;    // STM32 HAL I2C Handle
    uint16_t address;           // I2C Device Address (8-bit shifted)
    SemaphoreHandle_t lock;     // FreeRTOS Mutex for thread safety
    uint16_t yaw;      // Last read data
} WitMotion_Handle_t;

/* Function Prototypes */
HAL_StatusTypeDef WitMotion_Init(WitMotion_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef WitMotion_Update(WitMotion_Handle_t *dev);
void WitMotion_TaskEntry(void *argument);

/**
 * @brief  Thread-safe getter for the latest yaw data in degrees.
 * @param  dev Pointer to the driver handle.
 * @return The yaw angle in degrees (-180 to 180).
 */
float WitMotion_GetYaw_Degrees(WitMotion_Handle_t *dev);

/**
 * @brief  Thread-safe getter for the raw sensor value.
 * @note   It's better to use the function that returns degrees.
 * @param  dev Pointer to the driver handle.
 * @return The raw 16-bit signed integer value from the sensor.
 */
int16_t WitMotion_GetRawData(WitMotion_Handle_t *dev);

#ifdef __cplusplus
}
#endif

#endif // WITMOTION_HWT101_H
