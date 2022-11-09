/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <lis2dw12_reg.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_DELAY 1000 // 1s
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId VisualTaskHandle;
osThreadId AcceleroTaskHandle;
osMessageQId xVisualQueueHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartVisualTask(void const * argument);
void StartAcceleroTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t const *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static stmdev_ctx_t lis2dw12 = {
		.write_reg = platform_write,
		.read_reg = platform_read,
		.handle = &hi2c1,
};
/*
 * Replace the functions "platform_write" and "platform_read" with your
 * platform specific read and write function.
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
	return 0;
}
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}
int _write(int file, char const *buf, int n) { // this function is defined in stdio.h as a weak, so we rewrite it
	/* stdout redirection to UART2 */
	HAL_UART_Transmit(&huart2, (uint8_t*)(buf), n, HAL_MAX_DELAY);
	return n;
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of xVisualQueue */
  osMessageQDef(xVisualQueue, 16, uint16_t);
  xVisualQueueHandle = osMessageCreate(osMessageQ(xVisualQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of VisualTask */
  osThreadDef(VisualTask, StartVisualTask, osPriorityNormal, 0, 128);
  VisualTaskHandle = osThreadCreate(osThread(VisualTask), NULL);

  /* definition and creation of AcceleroTask */
  osThreadDef(AcceleroTask, StartAcceleroTask, osPriorityNormal, 0, 128);
  AcceleroTaskHandle = osThreadCreate(osThread(AcceleroTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart(); // We won't reach while and after this startup we will never reach even int main function

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartVisualTask */
/**
* @brief Function implementing the VisualTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVisualTask */
void StartVisualTask(void const * argument)
{
	/* USER CODE BEGIN StartVisualTask */
	/* Infinite loop */
	while(1)
	{
//		int16_t msg; TASK 1
//		if (xQueueReceive(xVisualQueueHandle, &msg, portMAX_DELAY)) { // reads msg from queue
//			if ((msg == -5000) || (msg == 5000)) { // according to the msg value is chosen between turning LED1/2 ON and LED2/1 OFF
//				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0); // LED2 OFF
//				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1); // LED1 ON
//			}
//			else {
//				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0); // LED1 OFF
//				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1); // LED2 ON
//			}
//			osDelay(1);
//		}
		int16_t msg;
		if (xQueueReceive(xVisualQueueHandle, &msg, portMAX_DELAY)) { // reads msg from queue
			if (msg >= 1000) { // according to the msg value is chosen between turning LED1/2 ON and LED2/1 OFF
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0); // LED1 OFF
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1); // LED2 ON
			}
			else if (msg <= -1000) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0); // LED2 OFF
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1); // LED1 ON
			}
			else {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0); // LED1 OFF
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0); // LED2 OFF
			}
			osDelay(1);
		}
		/* USER CODE END StartVisualTask */
	}
}

/* USER CODE BEGIN Header_StartAcceleroTask */
/**
* @brief Function implementing the AcceleroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAcceleroTask */
void StartAcceleroTask(void const * argument)
{
  /* USER CODE BEGIN StartAcceleroTask */
	// Necessary functions for lis2dw12 sensor driver startup
	lis2dw12_full_scale_set(&lis2dw12, LIS2DW12_2g);
	lis2dw12_power_mode_set(&lis2dw12, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2);
	lis2dw12_block_data_update_set(&lis2dw12, PROPERTY_ENABLE);
	lis2dw12_fifo_mode_set(&lis2dw12, LIS2DW12_STREAM_MODE); // enable continuous FIFO
	lis2dw12_data_rate_set(&lis2dw12, LIS2DW12_XL_ODR_25Hz); // enable part from power-down

	uint8_t samples;
	int16_t raw_acceleration[3];

	int16_t msg;

	// Check device ID - if I2C works correctly, OK is being sent to the UART
	uint8_t whoamI = 0;
	lis2dw12_device_id_get(&lis2dw12, &whoamI);
	printf("LIS2DW12_ID %s\n", (whoamI == LIS2DW12_ID) ? "OK" : "FAIL");

  /* Infinite loop */
	while(1)
	{
//		/* START OF IF CONDITIONS */ TASK 1
//		// they sets every time different value to msg
//		static uint8_t counter = 0;
//		if (counter == 0) {
//			counter++;
//			msg = -5000;
//		}
//		else if (counter == 1) {
//			counter++;
//			msg = 0;
//		}
//		else if (counter == 2) {
//			counter++;
//			msg = 5000;
//		}
//		else {
//			counter = 0;
//			msg = 0;
//		}
//		/* END OF IF CONDITIONS */
//		xQueueSend(xVisualQueueHandle, &msg, 0); // sends to the queue msg value
//		osDelay(300); // RTOS delay - blocking only in this process, but not in overall CPU time

		/* START OF TASK 2 */
		static uint32_t LastTicks = 0; // for non-blocking
		// this gets data from accelerometer including for cycle, which extracts all of the data
		lis2dw12_fifo_data_level_get(&lis2dw12, &samples);
		for (uint8_t i = 0; i < samples; i++) {
			// Read acceleration data
			lis2dw12_acceleration_raw_get(&lis2dw12, raw_acceleration);
		}
		// X coordinate is set to the msg
		msg = raw_acceleration[0];
		// sends X coordinate (msg) to the queue
		xQueueSend(xVisualQueueHandle, &msg, 0); // sends to the queue msg value

		if (xTaskGetTickCount() >= LastTicks + UART_DELAY) { // non-blocking waiting exclusive for RTOS
			LastTicks = xTaskGetTickCount(); // tick exclusive for RTOS
			printf("X=%d Y=%d Z=%d\n", raw_acceleration[0], raw_acceleration[1], raw_acceleration[2]); // sends all coordinates to UART
		}
		osDelay(50);
		/* END OF TASK 2 */
	}
  /* USER CODE END StartAcceleroTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
