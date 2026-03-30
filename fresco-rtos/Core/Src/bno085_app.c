#include "bno085_app.h"
#include "sh2.h"
#include "sh2_err.h"             // Needed for SH2_OK
#include "sh2_SensorValue.h"
#include "euler.h"               // For q_to_yaw()
#include "i2c.h"                 // Your STM32 I2C handle (e.g., hi2c1)
#include "main.h"                // For GPIO Pins
#include <string.h>              // Needed for memset()

// Native FreeRTOS Includes
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "ros_bridge.h"

// Typically 0x4A (if SA0=0) or 0x4B (if SA0=1). STM32 HAL requires it shifted by 1.
#define BNO_I2C_ADDR (0x4B << 1)

// Global HAL struct used by the library
sh2_Hal_t bno_hal;

// Thread-safe heading variable
float g_latest_yaw_deg = 0.0f;
static SemaphoreHandle_t g_yaw_mutex = NULL;

extern osMessageQueueId_t rosBridgeQueueHandle;


void I2C1_ClearBusyFlagErratum(void);


// --- 1. SH2 HAL INTERFACE IMPLEMENTATION ---

static int bno_hal_open(sh2_Hal_t *self) {
    // The reset pin is hard-pulled high, so we just delay to let it boot
    vTaskDelay(pdMS_TO_TICKS(250));
    return SH2_OK;
}

static void bno_hal_close(sh2_Hal_t *self) {
    // Nothing to de-init
}

static int bno_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    // Check if the BNO085 INT pin is pulled LOW (Data Ready)
    // Note: Replace BNO_INT_GPIO_Port / BNO_INT_Pin with your actual labels from CubeMX
    if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_SET) {
        return 0; // No data ready
    }

    uint8_t header[4];
    // Read the 4-byte SHTP header first
    if (HAL_I2C_Master_Receive(&hi2c1, BNO_I2C_ADDR, header, 4, 10) != HAL_OK) {
        return 0;
    }

    // Determine the packet length from the header (Drop MSB which is continuation bit)
    uint16_t packet_size = (header[0] | (header[1] << 8)) & 0x7FFF;
    if (packet_size == 0 || packet_size > len) {
        return 0;
    }

    // BNO085 allows us to re-read the entire packet (including header) in the next transaction
    if (HAL_I2C_Master_Receive(&hi2c1, BNO_I2C_ADDR, pBuffer, packet_size, 20) == HAL_OK) {
        *t_us = HAL_GetTick() * 1000;
        return packet_size;
    }

    return 0;
}

static int bno_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (HAL_I2C_Master_Transmit(&hi2c1, BNO_I2C_ADDR, pBuffer, len, 20) == HAL_OK) {
        return len;
    }
    return 0;
}

static uint32_t bno_hal_getTimeUs(sh2_Hal_t *self) {
    return HAL_GetTick() * 1000; // Return microsecond estimate
}


// --- 2. SENSOR CALLBACKS ---

static void sh2_event_handler(void *cookie, sh2_AsyncEvent_t *pEvent) {
    // Handles hub resets or errors (can leave empty for now)
}

static void sh2_sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent) {
    sh2_SensorValue_t value;

    // Decode the raw bytes into a C structure
    if (sh2_decodeSensorEvent(&value, pEvent) == SH2_OK) {

        // Only look at the Game Rotation Vector
        if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float r = value.un.gameRotationVector.real;
            float i = value.un.gameRotationVector.i;
            float j = value.un.gameRotationVector.j;
            float k = value.un.gameRotationVector.k;

            // CEVA euler.c returns radians (-pi to +pi)
            float yaw_rad = q_to_yaw(r, i, j, k);
            float yaw_deg = yaw_rad * (180.0f / 3.14159265f);

            // Keep output bounded 0-360
            if (yaw_deg < 0) yaw_deg += 360.0f;

            // Update thread-safe variable
            if (xSemaphoreTake(g_yaw_mutex, portMAX_DELAY) == pdTRUE) {
                g_latest_yaw_deg = yaw_deg;
                xSemaphoreGive(g_yaw_mutex);
            }

            // Push to ROS Bridge Queue
            BridgePacket_t tx_pkt;
            tx_pkt.type = MSG_YAW;
            tx_pkt.data.yaw = (uint16_t)yaw_deg;

            // Use a 0 timeout to avoid blocking the IMU task if the UART bridge is slow
            osMessageQueuePut(rosBridgeQueueHandle, &tx_pkt, 0, 0);
        }
    }
}


// --- 3. PUBLIC FUNCTIONS ---

// Call this to get the thread-safe Yaw (drop-in replacement for your WitMotion function)
uint16_t BNO085_GetYaw_Degrees(void) {
    float yaw = 0;
    if (g_yaw_mutex != NULL && xSemaphoreTake(g_yaw_mutex, portMAX_DELAY) == pdTRUE) {
        yaw = g_latest_yaw_deg;
        xSemaphoreGive(g_yaw_mutex);
    }
    return (uint16_t)yaw;
}


// The FreeRTOS Task Entry Point
void bno085TaskEntry(void *argument) {
	I2C1_ClearBusyFlagErratum();
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);

    g_yaw_mutex = xSemaphoreCreateMutex();

    // 1. Link the HAL structure
    bno_hal.open = bno_hal_open;
    bno_hal.close = bno_hal_close;
    bno_hal.read = bno_hal_read;
    bno_hal.write = bno_hal_write;
    bno_hal.getTimeUs = bno_hal_getTimeUs;

    // 2. Open SH2
    sh2_open(&bno_hal, sh2_event_handler, NULL);
    sh2_setSensorCallback(sh2_sensor_handler, NULL);

    // 3. Enable Game Rotation Vector (No magnetometer dependence, better for motors/indoors)
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // Request updates at 100Hz (10,000 microseconds)
    config.reportInterval_us = 10000;

    int status = sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
    (void)status;

    // 4. Infinite Processing Loop
    for(;;) {
        // Poll the SHTP protocol processor
        sh2_service();

        // Yield to the scheduler.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}









/**
 * @brief  Recovers the I2C bus from a stuck state.
 * @note   This is done by manually toggling the SCL line to force the
 *         slave to release the SDA line.
 */
void I2C1_ClearBusyFlagErratum(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. De-init I2C
    HAL_I2C_DeInit(&hi2c1);

    // 2. Configure SCL and SDA as GPIO Open-Drain
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // Assuming GPIOB for your pins

    // 3. Manually clock SCL 9 times
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1); // Small delay
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // 4. Generate a STOP condition
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    // 5. The bus should now be free. The pins will be reconfigured by HAL_I2C_Init().
}
