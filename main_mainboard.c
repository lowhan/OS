/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
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
/* Definitions for pumpA */
osThreadId_t pumpAHandle;
const osThreadAttr_t pumpA_attributes = {
  .name = "pumpA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pumpB */
osThreadId_t pumpBHandle;
const osThreadAttr_t pumpB_attributes = {
  .name = "pumpB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pumpC */
osThreadId_t pumpCHandle;
const osThreadAttr_t pumpC_attributes = {
  .name = "pumpC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pumpD */
osThreadId_t pumpDHandle;
const osThreadAttr_t pumpD_attributes = {
  .name = "pumpD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SemA */
osSemaphoreId_t SemAHandle;
const osSemaphoreAttr_t SemA_attributes = {
  .name = "SemA"
};
/* Definitions for SemB */
osSemaphoreId_t SemBHandle;
const osSemaphoreAttr_t SemB_attributes = {
  .name = "SemB"
};
/* Definitions for SemC */
osSemaphoreId_t SemCHandle;
const osSemaphoreAttr_t SemC_attributes = {
  .name = "SemC"
};
/* Definitions for SemD */
osSemaphoreId_t SemDHandle;
const osSemaphoreAttr_t SemD_attributes = {
  .name = "SemD"
};
/* USER CODE BEGIN PV */
static bool pumpAflag = false;
static bool pumpBflag = false;
static bool pumpCflag = false;
static bool pumpDflag = false;
int pumpAcount = 0;
int pumpBcount = 0;
int pumpCcount = 0;
int pumpDcount = 0;
int fuel = 50000;
__IO uint32_t tms = 0;

// ISR time
float fu0 = 0.0;		// in micro second
uint32_t n0 = 0;
__IO uint32_t t0 = 0;

float fu1 = 0.0;
uint32_t n1 = 0;
__IO uint32_t t1 = 0;

float it = 0.0;

// Thread time
float et0 = 0.0;		// in milli second
uint32_t m0 = 0;
__IO uint32_t u0 = 0;

float et1 = 0.0;
uint32_t m1 = 0;
__IO uint32_t u1 = 0;

float tt = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void pumpA_Thread(void *argument);
void pumpB_Thread(void *argument);
void pumpC_Thread(void *argument);
void pumpD_Thread(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(char message);

//int _write(int file, char *ptr, int len)
//{
//  (void)file;
//  int DataIdx;
//
//  for (DataIdx = 0; DataIdx < len; DataIdx++)
//  {
//	  ITM_SendChar(*ptr++);
//  }
//  return len;
//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	m = HAL_GetTick();
//	u = tms - SysTick->VAL;
//	et = m * 1000 + (u * 1000) / tms;
//	// Pump A callback
//	if(GPIO_Pin == GPIO_PIN_1)
//	{
//		pumpAflag = true;
//	}
//	else if(GPIO_Pin == GPIO_PIN_0)
//	{
//		pumpBflag = true;
//	}
//	else if(GPIO_Pin == GPIO_PIN_3)
//	{
//		pumpCflag = true;
//	}
//	else if(GPIO_Pin == GPIO_PIN_2)
//	{
//		pumpDflag = true;
//	}
//}

void EXTI0_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;
		pumpBflag = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
		n1 = HAL_GetTick();
		t1 = tms - SysTick->VAL;
		fu1 = n1 * 1000000 + (t1 * 1000000) / tms;
		it = fu1 - fu0;
	}
}

void EXTI1_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
	{
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;
		pumpAflag = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		n1 = HAL_GetTick();
		t1 = tms - SysTick->VAL;
		fu1 = n1 * 1000000 + (t1 * 1000000) / tms;
		it = fu1 - fu0;
	}
}

void EXTI2_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
	{
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;
		pumpDflag = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
		n1 = HAL_GetTick();
		t1 = tms - SysTick->VAL;
		fu1 = n1 * 1000000 + (t1 * 1000000) / tms;
		it = fu1 - fu0;
	}
}

void EXTI3_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET)
	{
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;
		pumpCflag = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
		n1 = HAL_GetTick();
		t1 = tms - SysTick->VAL;
		fu1 = n1 * 1000000 + (t1 * 1000000) / tms;
		it = fu1 - fu0;
	}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemA */
  SemAHandle = osSemaphoreNew(1, 1, &SemA_attributes);

  /* creation of SemB */
  SemBHandle = osSemaphoreNew(1, 0, &SemB_attributes);

  /* creation of SemC */
  SemCHandle = osSemaphoreNew(1, 0, &SemC_attributes);

  /* creation of SemD */
  SemDHandle = osSemaphoreNew(1, 0, &SemD_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of pumpA */
  pumpAHandle = osThreadNew(pumpA_Thread, NULL, &pumpA_attributes);

  /* creation of pumpB */
  pumpBHandle = osThreadNew(pumpB_Thread, NULL, &pumpB_attributes);

  /* creation of pumpC */
  pumpCHandle = osThreadNew(pumpC_Thread, NULL, &pumpC_attributes);

  /* creation of pumpD */
  pumpDHandle = osThreadNew(pumpD_Thread, NULL, &pumpD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CD_out_GPIO_Port, CD_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AB_out_GPIO_Port, AB_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pumpB_Pin pumpA_Pin pumpD_Pin pumpC_Pin */
  GPIO_InitStruct.Pin = pumpB_Pin|pumpA_Pin|pumpD_Pin|pumpC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CD_out_Pin */
  GPIO_InitStruct.Pin = CD_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CD_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AB_out_Pin */
  GPIO_InitStruct.Pin = AB_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AB_out_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void Task_action(char message)
{
	ITM_SendChar(message);
	ITM_SendChar('\n');
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_pumpA_Thread */
/**
  * @brief  Function implementing the pumpA thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pumpA_Thread */
void pumpA_Thread(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for(;;)
	{
		tms = SysTick->LOAD + 1;
		m0 = HAL_GetTick();
		u0 = tms - SysTick->VAL;
		et0 = m0 * 1000 + (u0 * 1000) / tms;
		osSemaphoreAcquire(SemAHandle, osWaitForever);
		if(pumpAflag == true && fuel > 0)							// ready to send fuel
		{
			Task_action('A');
			pumpAflag = false;
			pumpAcount = pumpAcount + 1;
			fuel = fuel - 1;
		}
		else if(fuel == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		}
		osSemaphoreRelease(SemBHandle);
		m1 = HAL_GetTick();
		u1 = tms - SysTick->VAL;
		et1 = m1 * 1000 + (u1 * 1000) / tms;
		tt = et1 - et0;
		osThreadYield();
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pumpB_Thread */
/**
* @brief Function implementing the pumpB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pumpB_Thread */
void pumpB_Thread(void *argument)
{
  /* USER CODE BEGIN pumpB_Thread */
  /* Infinite loop */
	for(;;)
	{
		tms = SysTick->LOAD + 1;
		m0 = HAL_GetTick();
		u0 = tms - SysTick->VAL;
		et0 = m0 * 1000 + (u0 * 1000) / tms;
		osSemaphoreAcquire(SemBHandle, osWaitForever);
		if(pumpBflag == true && fuel > 0)							// ready to send fuel
		{
			Task_action('B');
			pumpBflag = false;
			pumpBcount = pumpBcount + 1;
			fuel = fuel - 1;
		}
		else if(fuel == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		}
		osSemaphoreRelease(SemCHandle);
		m1 = HAL_GetTick();
		u1 = tms - SysTick->VAL;
		et1 = m1 * 1000 + (u1 * 1000) / tms;
		tt = et1 - et0;
		osThreadYield();
	}
  /* USER CODE END pumpB_Thread */
}

/* USER CODE BEGIN Header_pumpC_Thread */
/**
* @brief Function implementing the pumpC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pumpC_Thread */
void pumpC_Thread(void *argument)
{
  /* USER CODE BEGIN pumpC_Thread */
  /* Infinite loop */
	for(;;)
	{
		tms = SysTick->LOAD + 1;
		m0 = HAL_GetTick();
		u0 = tms - SysTick->VAL;
		et0 = m0 * 1000 + (u0 * 1000) / tms;
		osSemaphoreAcquire(SemCHandle, osWaitForever);
		if(pumpCflag == true && fuel > 0)							// ready to send fuel
		{
			Task_action('C');
			pumpCflag = false;
			pumpCcount = pumpCcount + 1;
			fuel = fuel - 1;
		}
		else if(fuel == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		}
		osSemaphoreRelease(SemDHandle);
		m1 = HAL_GetTick();
		u1 = tms - SysTick->VAL;
		et1 = m1 * 1000 + (u1 * 1000) / tms;
		tt = et1 - et0;
		osThreadYield();
	}
  /* USER CODE END pumpC_Thread */
}

/* USER CODE BEGIN Header_pumpD_Thread */
/**
* @brief Function implementing the pumpD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pumpD_Thread */
void pumpD_Thread(void *argument)
{
  /* USER CODE BEGIN pumpD_Thread */
  /* Infinite loop */
	for(;;)
	{
		tms = SysTick->LOAD + 1;
		m0 = HAL_GetTick();
		u0 = tms - SysTick->VAL;
		et0 = m0 * 1000 + (u0 * 1000) / tms;
		osSemaphoreAcquire(SemDHandle, osWaitForever);
		if(pumpDflag == true && fuel > 0)							// ready to send fuel
		{
			Task_action('D');
			pumpDflag = false;
			pumpDcount = pumpDcount + 1;
			fuel = fuel - 1;
		}
		else if(fuel == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		}
		osSemaphoreRelease(SemAHandle);
		m1 = HAL_GetTick();
		u1 = tms - SysTick->VAL;
		et1 = m1 * 1000 + (u1 * 1000) / tms;
		tt = et1 - et0;
		osThreadYield();
	}
  /* USER CODE END pumpD_Thread */
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