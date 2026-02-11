
#include "ros_bridge.h"
#include "cobs.h"

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




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (g_ros_bridge_ptr != NULL && huart == g_ros_bridge_ptr->huart) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Use the handle stored in the global pointer
        xSemaphoreGiveFromISR(g_ros_bridge_ptr->tx_sem, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

