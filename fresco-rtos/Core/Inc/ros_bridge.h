
#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"

#define MAX_PACKET_SIZE 256
#define RX_BUFFER_SIZE  512

/**
 * @brief Driver Handle
 */
typedef struct {
    UART_HandleTypeDef *huart;  //STM32 HAL UART Handle
    SemaphoreHandle_t lock;		//TX Mutex
    SemaphoreHandle_t tx_sem;    //TX Completion

    //TX Members
    uint8_t tx_buffer[MAX_PACKET_SIZE];
    uint8_t encode_tx_buffer[MAX_PACKET_SIZE+2];

    // RX Members (Circular DMA)
    uint8_t  dma_rx_buffer[RX_BUFFER_SIZE]; // DMA writes here automatically
    uint16_t rx_tail;                            // Software reads from here

    // Packet Assembly (Temporary buffers)
    uint8_t  raw_packet_buffer[MAX_PACKET_SIZE + 2]; // Holds encoded data before 0x00
    uint16_t raw_packet_index;
    uint8_t  decoded_buffer[MAX_PACKET_SIZE];        // Holds final decoded data

} ROS_Bridge_Handle_t;


typedef enum {
    MSG_IR_SENSORS,
    MSG_YAW,
    MSG_ODOMETRY,
    MSG_DEBUG_TEXT
} BridgeMsgType_t;

typedef struct {
    BridgeMsgType_t type;
    union {
        uint16_t ir[16];
        uint16_t yaw;
        float odom[3];
        char text[32];
    } data;
} BridgePacket_t;


// --- Protocol Command Structs (Reference /docs/PROTOCOL.md) ---

// ID 0x50
typedef struct {
    float vel_x;
    float vel_y;
    float omega;
} Cmd_Vel_t;

// ID 0x51
typedef struct {
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
} Cmd_RPM_t;

// The central queue handle
extern osMessageQueueId_t rosBridgeQueueHandle;


/* Variable Handle */
extern ROS_Bridge_Handle_t *g_ros_bridge_ptr; //Hacky, but gets the job done


/* Function Prototypes */
HAL_StatusTypeDef ROS_Bridge_Init(ROS_Bridge_Handle_t *dev, UART_HandleTypeDef *huart);
HAL_StatusTypeDef ROS_Bridge_Send_Packet(ROS_Bridge_Handle_t *dev, uint8_t id, void* data, uint16_t data_len);
void ROS_Bridge_TaskEntry(void *argument);

/* RX Processing Function */
void ROS_Bridge_Process_RX(ROS_Bridge_Handle_t *dev);

/* Helper Function Prototypes */
static uint8_t calculate_crc8(const uint8_t* data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // ROS_BRIDGE_H
