#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

extern I2C_HandleTypeDef hi2c2;

// Registers
#define REG_DIST_HIGH   0x02
#define REG_ADDR_CONFIG 0x05  // Register to change I2C address
#define REG_COMMAND     0x10

// Commands
#define CMD_TRIG_MODE1_MM 0xBD
#define CMD_TRIG_MODE2_MM 0xBC
#define CMD_TRIG_MODE4_MM 0xB4

/**
 * @brief Reads distance from a specific sensor
 * @param address: 8-bit I2C address of the sensor
 * @return distance in mm, -1 for I2C error, -2 for Out of Range
 */
int16_t DYP_ReadDistance(uint8_t address);

HAL_StatusTypeDef DYP_Trigger(uint8_t address);
int16_t DYP_GetDistance(uint8_t address);


/**
 * @brief Changes the I2C address of a sensor
 * @note  Only connect ONE sensor to the bus when running this!
 * @param current_addr: The current 8-bit address (default 0xD0)
 * @param new_addr: The desired new 8-bit address
 * @return HAL Status
 */
HAL_StatusTypeDef DYP_SetAddress(uint8_t current_addr, uint8_t new_addr);

#endif
