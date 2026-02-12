#pragma once

#include "main.h" // For HAL and UART handles
#include "cmsis_os.h"
#include <stdbool.h>
#include <string.h>

#include "ros_bridge.h"
extern osMessageQueueId_t rosBridgeQueueHandle;

#pragma pack(push, 1)
typedef struct {
    uint16_t ir_sensors[16];
    uint16_t ultrasonics[4];
    uint32_t timestamp_ms;
    uint8_t sequence_num;
    uint8_t crc8;
} sensor_packet_t;
#pragma pack(pop)

typedef struct {
    uint32_t packets_received;
    uint32_t packets_lost;     // Calculated via sequence gaps
    uint32_t crc_errors;       // Corrupted data
    uint32_t dma_overflows;    // If task is too slow
    float avg_latency_ms;      // Time diff
} protocol_stats_t;

#define HEADER_BYTE 0xAA
#define FOOTER_BYTE 0x55
#define PACKET_SIZE sizeof(sensor_packet_t)
#define FRAME_SIZE (PACKET_SIZE + 2) // Header + Packet + Footer

// --- Public Functions ---

// Call this in main() before osKernelStart()
void fresco_init(UART_HandleTypeDef *huart, osThreadId_t parserTaskHandle, osMessageQueueId_t queueHandle);

// The actual task function (Called by FreeRTOS wrapper)
void fresco_parser_task_entry(void);

// Get stats
protocol_stats_t fresco_get_stats(void);
