/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_protocol.h"
#include "motor_driver.h"
#include "witmotion_hwt101.h"
#include "ros_bridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
WitMotion_Handle_t hwt101;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for parserTask */
osThreadId_t parserTaskHandle;
const osThreadAttr_t parserTask_attributes = {
  .name = "parserTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for bridgeTask */
osThreadId_t bridgeTaskHandle;
const osThreadAttr_t bridgeTask_attributes = {
  .name = "bridgeTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for rosBridgeQueue */
osMessageQueueId_t rosBridgeQueueHandle;
const osMessageQueueAttr_t rosBridgeQueue_attributes = {
  .name = "rosBridgeQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartParserTask(void *argument);
void StartMotorTask(void *argument);
void StartGyroTask(void *argument);
void StartBridgeTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (5, sizeof(sensor_packet_t), &sensorQueue_attributes);

  /* creation of rosBridgeQueue */
  rosBridgeQueueHandle = osMessageQueueNew (20, sizeof(BridgePacket_t), &rosBridgeQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of parserTask */
  parserTaskHandle = osThreadNew(StartParserTask, NULL, &parserTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

  /* creation of gyroTask */
  gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);

  /* creation of bridgeTask */
  bridgeTaskHandle = osThreadNew(StartBridgeTask, NULL, &bridgeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	sensor_packet_t packet;
	extern WitMotion_Handle_t hwt101;
	extern int16_t sensorVal;
  /* Infinite loop */
  for(;;)
  {
//	  if (osMessageQueueGet(sensorQueueHandle, &packet, NULL, osWaitForever) == osOK)
//	  {
//		  // --- Packet Received ---
//	      uint16_t front_sensor = packet.ir_sensors[0];
//	      protocol_stats_t current_stats = fresco_get_stats();
//
//      }
//	  // Move Forward at 1.0 m/s
//	        chassis_set_velocity(1.0f, 0.0f, 0.0f);
//	        osDelay(2000);
//
//	        // Slide Right at 0.5 m/s
//	        chassis_set_velocity(0.0f, 0.5f, 0.0f);
//	        osDelay(2000);
//
//	        // Spin in place (PI rad/s = 180 degrees/sec)
//	        chassis_set_velocity(0.0f, 0.0f, 3.14f);
//	        osDelay(1000);
//
//	        // Combine: Move Forward + Spin (Arc)
//	        chassis_set_velocity(0.5f, 0.0f, 1.0f);
//	        osDelay(2000);
//
//	        // Stop
//	        chassis_set_velocity(0.0f, 0.0f, 0.0f);
//	        osDelay(2000);
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartParserTask */
/**
* @brief Function implementing the parserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParserTask */
void StartParserTask(void *argument)
{
  /* USER CODE BEGIN StartParserTask */
	extern UART_HandleTypeDef huart3;
	fresco_init(&huart3, parserTaskHandle, sensorQueueHandle);
	/* Infinite loop in function */
	fresco_parser_task_entry();
  /* USER CODE END StartParserTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
	extern CAN_HandleTypeDef hcan1;
	motors_init(&hcan1);
	/* Infinite loop in function */
	motor_control_task_entry();
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */
	extern I2C_HandleTypeDef hi2c1;
	extern WitMotion_Handle_t hwt101;
	while (WitMotion_Init(&hwt101, &hi2c1, HWT101_I2C_ADDR_DEFAULT) != HAL_OK) {
	        // Initialization Error Loop
		osDelay(100);
	}
	/* Infinite loop in function*/
	WitMotion_TaskEntry(&hwt101);
  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartBridgeTask */
/**
* @brief Function implementing the bridgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBridgeTask */
void StartBridgeTask(void *argument)
{
  /* USER CODE BEGIN StartBridgeTask */
	static ROS_Bridge_Handle_t my_ros_bridge;
	extern UART_HandleTypeDef huart1;
	ROS_Bridge_Init(&my_ros_bridge, &huart1);
	ROS_Bridge_TaskEntry(&my_ros_bridge);
  /* USER CODE END StartBridgeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

