/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_protocol.h"
#include "ultrasonic.h"
#include <stdlib.h>
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

/* USER CODE BEGIN PV */
#define SENSOR_COUNT 16
volatile uint16_t ir_data[16];
volatile uint16_t ultrasonic_data[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Update(void);
void Ultrasonic_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  mosaic_send_init(&huart1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ir_data, SENSOR_COUNT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LED_Update();
	  Ultrasonic_Update();
	  if (mosaic_send_sensors(ir_data, ultrasonic_data)){

	  }
	  HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LED_Update(void){
	int min=16;
		  for(int i=0;i<SENSOR_COUNT;i++){
			  if(ir_data[i]<4095){
				  if(min==17){
					  min=i;
				  }
				  if (ir_data[i]<ir_data[min]){
					  min=i;
				  }
			  }
		  }
		  switch (min){
		  case 16:
			  break;
		  case 0:
			  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, RESET);
			  break;
		  case 1:
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
			  break;
		  case 2:
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
			  break;
		  case 3:
			  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
			  break;
		  case 4:
			  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
			  break;
		  case 5:
			  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, RESET);
			  break;
		  case 6:
			  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, RESET);
			  break;
		  case 7:
			  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, RESET);
			  break;
		  case 8:
			  HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, RESET);
			  break;
		  case 9:
			  HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, RESET);
			  break;
		  case 10:
			  HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, RESET);
			  break;
		  case 11:
			  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, RESET);
			  break;
		  case 12:
			  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, RESET);
			  break;
		  case 13:
			  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, RESET);
			  break;
		  case 14:
			  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, RESET);
			  break;
		  case 15:
			  HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, RESET);
			  break;
		  }
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, SET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
		  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, SET);
		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, SET);
		  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, SET);
		  HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, SET);
		  HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, SET);
		  HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, SET);
		  HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, SET);
		  HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, SET);
		  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, SET);
		  HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, SET);
		  HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, SET);
}




void Ultrasonic_Update(void)
{
    static uint32_t us_timer = 0;
    static uint8_t us_state = 0; // 0 = Trigger, 1 = Wait

    // Sensor addresses
    static const uint8_t us_addrs[4] = {0x80, 0xD0, 0xD6, 0xD2};

    uint32_t current_time = HAL_GetTick();

    switch(us_state) {
        case 0: // STATE: TRIGGER
            for(int i = 0; i < 4; i++) {
                DYP_Trigger(us_addrs[i]);
            }
            // Capture time and move to wait state
            us_timer = current_time;
            us_state = 1;
            break;

        case 1: // STATE: WAIT
            // Check if set time has passed
            if ((current_time - us_timer) >= 160) {
                // Time is up, read data
                for(int i = 0; i < 4; i++) {
                    int16_t val = DYP_GetDistance(us_addrs[i]);

                    if(val > 0) {
                        ultrasonic_data[i] = (uint16_t)val;
                    } else {
                        ultrasonic_data[i] = 0; // Error or Out of Range
                    }
                }
                // Go back to Trigger state
                us_state = 0;
            }
            break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
