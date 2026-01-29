#include "uart_protocol.h"

// --- Config ---
#define DMA_BUFFER_SIZE (FRAME_SIZE * 8) // Hold 8 frames in raw buffer

// --- Private Variables ---
static UART_HandleTypeDef *phuart;
static osThreadId_t task_handle;
static osMessageQueueId_t queue_handle;

static uint8_t dma_rx_buffer[DMA_BUFFER_SIZE];
static uint16_t read_ptr = 0; // Software pointer

static protocol_stats_t stats = {0};
static uint8_t expected_seq = 0;
static bool first_packet = true;

// --- Helper Prototypes ---
static uint8_t calculate_crc8(const uint8_t* data, size_t length);
static void process_valid_packet(const sensor_packet_t* pkt);

// --- Implementation ---

void fresco_init(UART_HandleTypeDef *huart, osThreadId_t parserTask, osMessageQueueId_t queue) {
    phuart = huart;
    task_handle = parserTask;
    queue_handle = queue;

    // Reset stats
    memset(&stats, 0, sizeof(stats));
    read_ptr = 0;
    first_packet = true;

    // Start DMA in Circular Mode
    HAL_UART_Receive_DMA(phuart, dma_rx_buffer, DMA_BUFFER_SIZE);
}

protocol_stats_t fresco_get_stats(void) {
    return stats;
}

// This is the "Brain". Put this inside the FreeRTOS Task wrapper in main.c
void fresco_parser_task_entry(void) {
    // Parser State
    static uint8_t frame_buf[FRAME_SIZE];
    static uint8_t frame_idx = 0;
    static bool seeking_header = true;

    while (1) {
        // 1. Wait efficiently for data (Notification from ISR)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2. Determine how much data DMA has written
        // CNDTR counts DOWN from Size to 0.
        uint16_t write_ptr = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(phuart->hdmarx);

        // 3. Process the Ring Buffer
        while (read_ptr != write_ptr) {
            uint8_t byte = dma_rx_buffer[read_ptr];

            // --- State Machine ---
            if (seeking_header) {
                if (byte == HEADER_BYTE) {
                    frame_buf[0] = byte;
                    frame_idx = 1;
                    seeking_header = false;
                }
            }
            else {
                frame_buf[frame_idx++] = byte;

                // Frame complete?
                if (frame_idx >= FRAME_SIZE) {
                    // Check Footer
                    if (frame_buf[FRAME_SIZE - 1] == FOOTER_BYTE) {
                        sensor_packet_t* pkt = (sensor_packet_t*)&frame_buf[1];

                        // Check CRC
                        uint8_t calced = calculate_crc8((uint8_t*)pkt, sizeof(sensor_packet_t) - 1);

                        if (calced == pkt->crc8) {
                            process_valid_packet(pkt); // Updates Stats & Pushes to Queue
                        } else {
                            stats.crc_errors++;
                        }
                    } else {
                        // Footer not found where expected
                        stats.crc_errors++;
                    }

                    // Reset for next packet
                    seeking_header = true;
                    frame_idx = 0;
                }
            }

            // Advance Read Pointer (Wrap around)
            read_ptr++;
            if (read_ptr >= DMA_BUFFER_SIZE) {
                read_ptr = 0;
            }
        }
    osDelay(1);
    }
}

static void process_valid_packet(const sensor_packet_t* pkt) {
    // 1. Update Statistics
    if (first_packet) {
        expected_seq = pkt->sequence_num + 1;
        first_packet = false;
    } else {
        uint8_t diff = pkt->sequence_num - expected_seq;
        // If diff is 0, exact match. If diff > 0, we skipped packets.
        if (diff > 0) {
            stats.packets_lost += diff;
        }
        expected_seq = pkt->sequence_num + 1;
    }

    stats.packets_received++;

    // Calculate Latency (Smoothing average)
    uint32_t now = HAL_GetTick();
    uint32_t latency = now - pkt->timestamp_ms;
    stats.avg_latency_ms = (stats.avg_latency_ms * 0.95f) + ((float)latency * 0.05f);

    // 2. Send to Queue
    // If Queue is full, we log an error, don't block the parser
    if (osMessageQueuePut(queue_handle, pkt, 0, 0) != osOK) {
        stats.dma_overflows++; // Using this field to indicate Queue full
    }
}

// --- Interrupt Callbacks ---
// These notify the task when DMA is Half-Full or Full
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == phuart) {
        vTaskNotifyGiveFromISR(task_handle, NULL);
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == phuart) {
        vTaskNotifyGiveFromISR(task_handle, NULL);
    }
}

uint8_t calculate_crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    while (length--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}
