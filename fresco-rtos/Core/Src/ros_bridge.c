
#include "ros_bridge.h"
#include "cobs.h"
#include <string.h>
#include <stdio.h>

#include "witmotion_hwt101.h"

// Helper to access the DMA instance (DMA2_Stream2)
#define GET_DMA_COUNTER(dev) __HAL_DMA_GET_COUNTER((dev)->huart->hdmarx)


extern void motors_set_rpm_manual(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
extern void chassis_set_velocity(float vx, float vy, float omega);



ROS_Bridge_Handle_t *g_ros_bridge_ptr = NULL;
HAL_StatusTypeDef res;
HAL_StatusTypeDef ROS_Bridge_Init(ROS_Bridge_Handle_t *dev, UART_HandleTypeDef *huart){
    if (dev == NULL || huart == NULL) {
        return HAL_ERROR;
    }

    g_ros_bridge_ptr = dev;

    dev->huart = huart;

    // Create a mutex for thread safety
    dev->lock = xSemaphoreCreateMutex();
    dev->tx_sem = xSemaphoreCreateBinary();

    // Reset RX indices
     dev->rx_tail = 0;
     dev->raw_packet_index = 0;

//     HAL_UARTEx_ReceiveToIdle_DMA(dev->huart, dev->dma_rx_buffer, RX_BUFFER_SIZE);
//     __HAL_DMA_DISABLE_IT(dev->huart->hdmarx, DMA_IT_HT);
     HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buffer, RX_BUFFER_SIZE);


    return (dev->lock && dev->tx_sem) ? HAL_OK : HAL_ERROR;
}


HAL_StatusTypeDef ROS_Bridge_Send_Packet(ROS_Bridge_Handle_t *dev, uint8_t id, void* data, uint16_t data_len){
	if (data_len + 2 > MAX_PACKET_SIZE) return HAL_ERROR; // Size check

	    // 1. Wait for the UART to be free from other tasks
	    if (xSemaphoreTake(dev->lock, pdMS_TO_TICKS(50)) == pdTRUE) {

	        // 2. Build Raw Packet: [ID][DATA][CRC]
	        dev->tx_buffer[0] = id;
	        memcpy(&dev->tx_buffer[1], data, data_len);

	        uint8_t crc = calculate_crc8(dev->tx_buffer, data_len + 1);
	        dev->tx_buffer[data_len + 1] = crc;

	        // 3. Encode with COBS
	        // (ID + DATA + CRC = data_len + 2 total bytes)
	        size_t encoded_len = cobs_encode(dev->tx_buffer, data_len + 2, dev->encode_tx_buffer);
	        dev->encode_tx_buffer[encoded_len] = 0x00; // Delimiter

	        // 4. Start Transmission
//	        res = HAL_UART_Transmit(dev->huart, dev->encode_tx_buffer, encoded_len+1, 9999);
	        HAL_UART_Transmit_DMA(dev->huart, dev->encode_tx_buffer, encoded_len + 1);

	        // 5. Wait for the ISR to signal completion
	        if (xSemaphoreTake(dev->tx_sem, pdMS_TO_TICKS(10)) == pdTRUE) {
	            // Success
	        } else {
	            // Timeout
	        	// Note how the function releases the UART mutex anyways
	        }

	        xSemaphoreGive(dev->lock); // Release the UART for the next caller
	        return HAL_OK;
	    }
	    return HAL_BUSY;
}


void ROS_Bridge_TaskEntry(void *argument){
	extern WitMotion_Handle_t hwt101;
	// 1. Recover the handle from the void pointer
	ROS_Bridge_Handle_t *dev = (ROS_Bridge_Handle_t *)argument;

	if (dev == NULL || dev->huart == NULL) {
	    // Error: Task started without a valid handle
	    vTaskDelete(NULL);
	}

	BridgePacket_t tx_buff;

	uint32_t last_telemetry_time = 0;

	// 2. Infinite Loop
	for(;;){
		// 1. Process incoming data from Pi
		ROS_Bridge_Process_RX(dev);

		// 2. Check Queue for outgoing messages (e.g. from IR Sensor Task)
		if (osMessageQueueGet(rosBridgeQueueHandle, &tx_buff, NULL, 2) == osOK){
			switch(tx_buff.type){
				case MSG_IR_SENSORS:
					ROS_Bridge_Send_Packet(dev, 0x10, &tx_buff.data.ir, 32);
					break;
//				case MSG_YAW:
//					ROS_Bridge_Send_Packet(dev, 0x11, &tx_buff, 6);
//					break;
			}
		}

		// 3. Periodic Telemetry (50ms / 20Hz)
        uint32_t now = osKernelGetTickCount();
        if ((now - last_telemetry_time) >= 50) { // 50ms = 20Hz
            last_telemetry_time = now;

            // Grab the latest values from the other modules
            uint32_t ts = HAL_GetTick();
            uint16_t current_yaw = WitMotion_GetData(&hwt101);
            uint8_t payload[6];
            // Pack 32-bit Timestamp (Little Endian)
            payload[0] = (uint8_t)(ts & 0xFF);
            payload[1] = (uint8_t)((ts >> 8) & 0xFF);
            payload[2] = (uint8_t)((ts >> 16) & 0xFF);
            payload[3] = (uint8_t)((ts >> 24) & 0xFF);

            // Pack 16-bit Yaw (Little Endian)
            payload[4] = (uint8_t)(current_yaw & 0xFF);
            payload[5] = (uint8_t)((current_yaw >> 8) & 0xFF);

            ROS_Bridge_Send_Packet(dev, 0x11, &payload, 6);
//            MotorState_t motors = Motor_GetState();

            // Send them straight to the UART
//            ROS_Bridge_Send_Packet(&dev, 0x13, &motors, sizeof(motors));
        }

	}
}



/**
 * @brief  Reads from DMA buffer, finds 0x00, decodes, and parses commands.
 * @note   Call this frequently (e.g., inside the infinite loop of the Task)
 */
void ROS_Bridge_Process_RX(ROS_Bridge_Handle_t *dev) {
	// 0. Check for Hardware Errors (ORE, FE, NE)
	    if (__HAL_UART_GET_FLAG(dev->huart, UART_FLAG_ORE) ||
	        __HAL_UART_GET_FLAG(dev->huart, UART_FLAG_FE)) {

	        __HAL_UART_CLEAR_OREFLAG(dev->huart);
	        __HAL_UART_CLEAR_FEFLAG(dev->huart);

	        // Force restart the DMA
	        HAL_UART_DMAStop(dev->huart);
	        HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buffer, RX_BUFFER_SIZE);

	        char* err = "UART_ERROR_RESET";
	        ROS_Bridge_Send_Packet(dev, 0x4F, (uint8_t*)err, 16);
	    }

    // 1. Calculate the Head (where DMA is currently writing)
    // CNT goes *down* from Size to 0 in STM32
    uint16_t rx_head = RX_BUFFER_SIZE - GET_DMA_COUNTER(dev);

    // 2. Process all new bytes
    while (dev->rx_tail != rx_head) {

        // Read one byte
        uint8_t byte_in = dev->dma_rx_buffer[dev->rx_tail];

        // Increment Tail (Circular Wrap)
        dev->rx_tail++;
        if (dev->rx_tail >= RX_BUFFER_SIZE) {
            dev->rx_tail = 0;
        }

        // 3. Packet Framing State Machine
        if (byte_in == 0x00) {
            // --- END OF PACKET FOUND ---

            // Need at least 2 bytes (ID + CRC) + overhead
            if (dev->raw_packet_index > 0) {

                // A. Decode COBS
                // cobs_decode(input, length, output) returning size_t
                size_t decoded_len = cobs_decode(dev->raw_packet_buffer,
                                                 dev->raw_packet_index,
                                                 dev->decoded_buffer);

                // B. Validate Integrity (CRC8)
                // We expect [ID][DATA...][CRC]
                // decoded_len includes ID and CRC.
                if (decoded_len >= 2) {
                    uint8_t received_crc = dev->decoded_buffer[decoded_len - 1];
                    uint8_t calc_crc = calculate_crc8(dev->decoded_buffer, decoded_len - 1);

                    if (received_crc == calc_crc) {
                        // --- PACKET VALID ---
                        uint8_t msg_id = dev->decoded_buffer[0];
                        uint8_t *payload = &dev->decoded_buffer[1];
                        // Payload length = Total - ID(1) - CRC(1)
                         uint16_t payload_len = decoded_len - 2;

//                        // --- DEBUG ECHO SNIPPET ---
//                        // This creates a string like "Recv: 0x50 Len: 12" and sends it to the Pi
//                        char debug_str[32];
//                        int str_len = snprintf(debug_str, sizeof(debug_str), "Recv: 0x%02X Len: %d", msg_id, payload_len);
//
//                        // Using ID 0x4F for Debug Text (as per your ID range 0x10-0x4F)
//                        ROS_Bridge_Send_Packet(dev, 0x4F, (uint8_t*)debug_str, str_len);

                        // Dispatch Command
                        switch (msg_id) {
                            case 0x50: { // Cmd_Vel
                                Cmd_Vel_t vel;
                                memcpy(&vel, payload, sizeof(Cmd_Vel_t));
                                chassis_set_velocity(vel.vel_x, vel.vel_y, vel.omega);
                                break;
                            }
                            case 0x51: { // Cmd_RPM
                                Cmd_RPM_t rpm;
                                memcpy(&rpm, payload, sizeof(Cmd_RPM_t));
                                motors_set_rpm_manual(rpm.m1, rpm.m2, rpm.m3, rpm.m4);
                                break;
                            }
                            // Add more cases here
                        }
                    }
                }
            }

            // Reset packet index to start receiving the next one
            dev->raw_packet_index = 0;

        } else {
            // --- ACCUMULATE BYTES ---
            if (dev->raw_packet_index < MAX_PACKET_SIZE) {
                dev->raw_packet_buffer[dev->raw_packet_index++] = byte_in;
            } else {
                // Buffer Overflow: Packet is too big or garbage data. Reset.
                dev->raw_packet_index = 0;
            }
        }
    }
}


static uint8_t calculate_crc8(const uint8_t* data, size_t length) {
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




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (g_ros_bridge_ptr != NULL && huart == g_ros_bridge_ptr->huart) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Use the handle stored in the global pointer
        xSemaphoreGiveFromISR(g_ros_bridge_ptr->tx_sem, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

