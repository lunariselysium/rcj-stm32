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
#include "bno085_app.h"
#include "ros_bridge.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
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
// Strategy constants
#define IR_SENSOR_COUNT 16
#define IR_SENSOR_ANGLE_DEG 22.5f  // degrees between sensors (sensor 0 front, CCW)
#define IR_THRESHOLD 4000          // ADC value below which ball is detected (lower = stronger)
#define BALL_LOST_TIMEOUT_MS 1000   // use last known ball direction for this long
#define ALIGN_THRESHOLD_RAD 0.4f   // ~5.7 degrees
#define SEARCH_SPIN_SPEED 0.5f     // rad/s
#define ALIGN_KP 1.8f              // proportional gain for alignment
#define MAX_OMEGA 3.0f             // maximum angular speed (rad/s)
#define DRIVE_SPEED 0.6f           // m/s
#define M_PI 3.14159265358979323846f

// Helper macro to clamp a value between min and max
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

typedef enum {
    STRATEGY_SEARCH,
    STRATEGY_ALIGN,
    STRATEGY_DRIVE
} strategy_state_t;

static strategy_state_t strategy_state = STRATEGY_SEARCH;
static float last_ball_direction_rad = 0.0f;  // radians, 0 = front, positive CCW
static uint32_t ball_last_seen_time = 0;      // HAL_GetTick() when ball was last detected
static bool ball_currently_detected = false;
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
	uint32_t current_time;
	float ball_direction_rad;
	bool ball_detected;
  /* Infinite loop */
  for(;;)
  {
	    current_time = HAL_GetTick();

	    // 1. Drain the queue to get the FRESHEST sensor data
	    // Using 'while' ensures we process all backlogged messages to eliminate lag
	    while (osMessageQueueGet(sensorQueueHandle, &packet, NULL, 0) == osOK)
	    {
	      uint16_t min_adc = 4095;
	      uint8_t min_idx = 0;
	      for (int i = 0; i < IR_SENSOR_COUNT; i++)
	      {
	        if (packet.ir_sensors[i] < min_adc)
	        {
	          min_adc = packet.ir_sensors[i];
	          min_idx = i;
	        }
	      }

	      // If we see the ball in this fresh packet, update the timestamp and angle
	      if (min_adc < IR_THRESHOLD)
	      {
	        ball_last_seen_time = current_time;

	        // Vector sum weighted by reflection strength for robust direction
	        float sum_x = 0.0f;
	        float sum_y = 0.0f;
	        float total_weight = 0.0f;
	        for (int i = 0; i < IR_SENSOR_COUNT; i++)
	        {
	          if (packet.ir_sensors[i] < IR_THRESHOLD)
	          {
	            float weight = 4095.0f - packet.ir_sensors[i];
	            float angle_deg = i * IR_SENSOR_ANGLE_DEG;
	            float angle_rad = angle_deg * (M_PI / 180.0f);
	            sum_x += weight * cos(angle_rad);
	            sum_y += weight * sin(angle_rad);
	            total_weight += weight;
	          }
	        }
	        if (total_weight > 0.0f)
	        {
	          last_ball_direction_rad = atan2(sum_y, sum_x);
	        }
	        else
	        {
	          // fallback to strongest sensor
	          last_ball_direction_rad = min_idx * IR_SENSOR_ANGLE_DEG * (M_PI / 180.0f);
	          while (last_ball_direction_rad > M_PI) last_ball_direction_rad -= 2 * M_PI;
	          while (last_ball_direction_rad < -M_PI) last_ball_direction_rad += 2 * M_PI;
	        }
	      }
	    }

	    // 2. Single Source of Truth for Ball Detection (The Timeout)
	    // Make sure ball_last_seen_time != 0 so it doesn't falsely trigger at startup
	    if (ball_last_seen_time != 0 && (current_time - ball_last_seen_time < BALL_LOST_TIMEOUT_MS))
	    {
	      ball_detected = true;
	      ball_direction_rad = last_ball_direction_rad;
	    }
	    else
	    {
	      ball_detected = false;
	    }
    
    // State machine
    switch (strategy_state)
        {
          case STRATEGY_SEARCH:
            // If ball is seen, move to align
            if (ball_detected)
            {
              strategy_state = STRATEGY_ALIGN;
            }
            else
            {
              // Spin slowly to find ball
              chassis_set_velocity(0.0f, 0.0f, SEARCH_SPIN_SPEED);
            }
            break;

          case STRATEGY_ALIGN:
            if (ball_detected)
            {
              float error_rad = ball_direction_rad; // 0 = front, positive CCW

              // Proportional controller for rotation
              float omega = CLAMP(ALIGN_KP * error_rad, -MAX_OMEGA, MAX_OMEGA);
              chassis_set_velocity(0.0f, 0.0f, omega);

              // Check if aligned (error within threshold)
              if (fabs(error_rad) < ALIGN_THRESHOLD_RAD)
              {
                strategy_state = STRATEGY_DRIVE;
              }
            }
            else
            {
              // Timeout expired, ball completely lost
              strategy_state = STRATEGY_SEARCH;
            }
            break;

          case STRATEGY_DRIVE:
            if (ball_detected)
            {
              float error_rad = ball_direction_rad;

              // Hysteresis: If ball drifts too far off-center, stop driving and realign
              if (fabs(error_rad) > ALIGN_THRESHOLD_RAD * 2.0f)
              {
                strategy_state = STRATEGY_ALIGN;
              }
              else
              {
                // Continuously adjust heading (omega) while driving forward
                float omega = CLAMP(ALIGN_KP * error_rad, -MAX_OMEGA, MAX_OMEGA);
                chassis_set_velocity(DRIVE_SPEED, 0.0f, omega);
              }
            }
            else
            {
              // Timeout expired, ball completely lost
              strategy_state = STRATEGY_SEARCH;
            }
            break;
    }
    
    osDelay(20); // 50 Hz strategy loop
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
//	extern I2C_HandleTypeDef hi2c1;
	bno085TaskEntry(argument);
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

