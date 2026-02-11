
#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define MAX_PACKET_SIZE 256

/**
 * @brief Driver Handle
 */
typedef struct {
    UART_HandleTypeDef *huart;  //STM32 HAL UART Handle
    SemaphoreHandle_t lock;
    SemaphoreHandle_t tx_sem;    // Binary Semaphore as Completion Signal
    uint8_t tx_buffer[MAX_PACKET_SIZE];
    uint8_t encode_tx_buffer[MAX_PACKET_SIZE+2];

} ROS_Bridge_Handle_t;


/* Variable Handle */
extern ROS_Bridge_Handle_t *g_ros_bridge_ptr; //Hacky, but gets the job done


/* Function Prototypes */
HAL_StatusTypeDef ROS_Bridge_Init(ROS_Bridge_Handle_t *dev, UART_HandleTypeDef *huart);
HAL_StatusTypeDef ROS_Bridge_Send_Packet(ROS_Bridge_Handle_t *dev, uint8_t id, void* data, uint16_t data_len);
void ROS_Bridge_TaskEntry(void *argument);

/* Helper Function Prototypes */
uint8_t calculate_crc8(const uint8_t* data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // ROS_BRIDGE_H
