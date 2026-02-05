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
#define WIT_REG_ANG_YAW 0x3F

/**
 * @brief Driver Handle
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;    // STM32 HAL I2C Handle
    uint16_t address;           // I2C Device Address (8-bit shifted)
    SemaphoreHandle_t lock;     // FreeRTOS Mutex for thread safety
    uint8_t yaw;      // Last read data
} WitMotion_Handle_t;

/* Function Prototypes */
HAL_StatusTypeDef WitMotion_Init(WitMotion_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef WitMotion_Update(WitMotion_Handle_t *dev);
uint16_t WitMotion_GetData(WitMotion_Handle_t *dev);
void WitMotion_TaskEntry(void *argument);

#ifdef __cplusplus
}
#endif

#endif // WITMOTION_HWT101_H
