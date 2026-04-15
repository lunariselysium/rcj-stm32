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
#define SEARCH_SPIN_SPEED 0.2f     // rad/s
#define ALIGN_KP 1.5f              // proportional gain for alignment
#define MAX_OMEGA 2.0f             // maximum angular speed (rad/s)
#define DRIVE_SPEED 0.4f           // m/s
#define HEADING_KP 2.0f                // Proportional gain for maintaining heading (TUNE THIS!)
#define PUSH_ENTRY_BALL_ANGLE_RAD 0.2f // ~11.5 degrees. Ball must be within this angle in front.
#define PUSH_ENTRY_YAW_ANGLE_RAD 0.2f  // ~11.5 degrees. Robot must be facing goal within this angle.
#define PUSH_EXIT_BALL_ANGLE_RAD 0.6f  // ~34 degrees. If ball angle exceeds this, stop pushing. (TUNE THIS!)
#define M_PI 3.14159265358979323846f

// Helper macro to clamp a value between min and max
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

typedef enum {
    STRATEGY_SEARCH,
    STRATEGY_ALIGN,
    STRATEGY_DRIVE,
	STRATEGY_PUSH
} strategy_state_t;

static strategy_state_t strategy_state = STRATEGY_SEARCH;
static float last_ball_direction_rad = 0.0f;  // radians, 0 = front, positive CCW
static uint32_t ball_last_seen_time = 0;      // HAL_GetTick() when ball was last detected

sensor_packet_t packet;
static volatile float yaw_rad;
static float global_ball_heading;
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
//	  sensor_packet_t packet;
	  uint32_t current_time;
	  float target_angle = 0.0f;
	  bool ball_detected = false;
	  static bool is_orbiting = false; // Tracks if we are circling behind the ball
	  static bool orbit_left = false;

	  // External declaration for IMU function
	  extern uint16_t BNO085_GetYaw_Degrees(void);

	  /* Infinite loop */
	  for(;;)
	  {
	    current_time = HAL_GetTick();

	    // 1. Drain the queue to get the FRESHEST sensor data (Zero Lag)
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

	      if (min_adc < IR_THRESHOLD)
	      {
	        ball_last_seen_time = current_time;

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
	          last_ball_direction_rad = min_idx * IR_SENSOR_ANGLE_DEG * (M_PI / 180.0f);
	          while (last_ball_direction_rad > M_PI) last_ball_direction_rad -= 2 * M_PI;
	          while (last_ball_direction_rad < -M_PI) last_ball_direction_rad += 2 * M_PI;
	        }
	      }
	    }

	    // 2. Ball logic & Goal Alignment
	    if (ball_last_seen_time != 0 && (current_time - ball_last_seen_time < BALL_LOST_TIMEOUT_MS))
	    {
	      ball_detected = true;

	      // Get Robot Heading (0 is opponent goal)
//	      float yaw_rad = BNO085_GetYaw_Degrees() * (M_PI / 180.0f);
	      yaw_rad = BNO085_GetYaw_Degrees() * (M_PI / 180.0f);
	      while (yaw_rad > M_PI) yaw_rad -= 2.0f * M_PI;
	      while (yaw_rad < -M_PI) yaw_rad += 2.0f * M_PI;

	      // Calculate Global Heading of the ball from the robot
//	      float global_ball_heading = yaw_rad + last_ball_direction_rad;
	      global_ball_heading = yaw_rad + last_ball_direction_rad;
	      while (global_ball_heading > M_PI) global_ball_heading -= 2.0f * M_PI;
	      while (global_ball_heading < -M_PI) global_ball_heading += 2.0f * M_PI;

	      // Hysteresis to switch smoothly between Attacking and Orbiting
	      float abs_gbh = fabs(global_ball_heading);
	      if (is_orbiting) {
	          // Stop orbiting once we are safely facing the opponent side
	          if (abs_gbh < (M_PI / 6.0f)) {
	              is_orbiting = false;
	          }
	      } else {
	          // Start orbiting if pushing would send it to our own half
	          if (abs_gbh > (M_PI / 3.0f)) {
	              is_orbiting = true;
	              // LOCK IN THE DIRECTION HERE:
	              // If the ball's global heading is positive, we commit to going Left.
	              orbit_left = (global_ball_heading > 0);
	          }
	      }

//	      target_angle = last_ball_direction_rad;
//
//	      if (is_orbiting) {
//	          // Add a 1.5 rad (~85 deg) offset to curve around the side of the ball
//	          if (global_ball_heading > 0) target_angle -= 1.5f;
//	          else target_angle += 1.5f;
//
//	          // Re-normalize target angle to bounds
//	          while (target_angle > M_PI) target_angle -= 2.0f * M_PI;
//	          while (target_angle < -M_PI) target_angle += 2.0f * M_PI;
//	      }
	    }
	    else
	    {
	      ball_detected = false;
	    }

	    // 3. State machine
	    switch (strategy_state)
	        {
	          case STRATEGY_SEARCH:
	            if (ball_detected) strategy_state = STRATEGY_ALIGN; // We'll repurpose ALIGN as TRACK
	            else chassis_set_velocity(0.0f, 0.0f, SEARCH_SPIN_SPEED);
	            break;

	          case STRATEGY_ALIGN: // Acts as TRACKING and ORBITING
	            if (ball_detected)
	            {
	              // 1. ALWAYS aim to face the ball directly
	              float error_rad = last_ball_direction_rad;
	              float omega = CLAMP(ALIGN_KP * error_rad, -MAX_OMEGA, MAX_OMEGA);

	              float vx = 0.0f;
	              float vy = 0.0f;

	              // 2. Decide movement based on Goal Alignment
	              if (is_orbiting) {
	                  vy = orbit_left ? DRIVE_SPEED : -DRIVE_SPEED;
	                  vx = 0.0f; // Perfect circle orbit
                      vy = vy*0.8;
                      omega = omega*1.45;
	                  chassis_set_velocity(vx, vy, omega);
	              } else {
	                  // ATTACK MODE
	                  // Check if we meet the conditions to start a straight push
	                  if (fabs(last_ball_direction_rad) < PUSH_ENTRY_BALL_ANGLE_RAD && fabs(yaw_rad) < PUSH_ENTRY_YAW_ANGLE_RAD)
	                  {
	                      // Conditions met: Ball is centered, robot is facing goal.
	                      strategy_state = STRATEGY_PUSH;
	                  }
	                  else
	                  {
	                      // Not aligned enough yet, continue driving towards the ball.
	                      vx = DRIVE_SPEED;
	                      chassis_set_velocity(vx, 0.0f, omega);
	                  }
	              }
	            }
	            else
	            {
	              strategy_state = STRATEGY_SEARCH;
	            }
	            break;

	          case STRATEGY_DRIVE:
	            // State completely removed to prevent fighting!
	            break;

	    	case STRATEGY_PUSH:
	    		if (ball_detected)
	    		        {
	    		            // Has the ball slipped out of the front of the robot?
	    		            if (fabs(last_ball_direction_rad) > PUSH_EXIT_BALL_ANGLE_RAD)
	    		            {
	    		                // The ball is still detected, but it's too far to the side.
	    		                // Go back to ALIGN to actively recover and re-center it.
	    		                strategy_state = STRATEGY_ALIGN;
	    		            }
	    		            else
	    		            {
	    		                // Ball is still securely in front. Continue the straight push.
	    		                // Goal: Maintain heading of 0 radians (straight at opponent goal)
	    		                float heading_error_rad = 0.0f + yaw_rad;

	    		                // P-controller to correct heading drift
	    		                float omega = CLAMP(HEADING_KP * heading_error_rad, -MAX_OMEGA, MAX_OMEGA);

	    		                // Push forward at full speed.
	    		                chassis_set_velocity(DRIVE_SPEED, 0.0f, omega);
	    		            }
	    		        }
	    		        else
	    		        {
	    		            // Ball has been completely lost, go back to searching for it.
	    		            strategy_state = STRATEGY_SEARCH;
	    		        }
	    		        break;
	    }

	    osDelay(20);
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

