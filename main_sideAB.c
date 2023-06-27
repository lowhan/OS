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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static bool pumpAflag = false;
static bool pumpBflag = false;
static bool pumpstop = false;
int pumpAcurrent = 0;
int pumpBcurrent = 0;
int pumpAprevious = 0;
int pumpBprevious = 0;
int pumpAcount = 0;
int pumpBcount = 0;
int current = 0;
int previous = 0;
int timeignore = 50;

__IO uint32_t tms = 0;

// ISR time
float fu0 = 0.0;
uint32_t n0 = 0;
__IO uint32_t t0 = 0;

float fu1 = 0.0;
uint32_t n1 = 0;
__IO uint32_t t1 = 0;

float it = 0.0;
int pumpdifference = 260;  //111 115*2
int pulsedelay = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	pumpAcurrent = pumpBcurrent = HAL_GetTick();
//
//	// Pump A callback
//	if((pumpAcurrent - pumpAprevious) > timeignore)
//	{
//		pumpAprevious = pumpAcurrent;
//		if(GPIO_Pin == GPIO_PIN_1 && pumpAflag == false)
//		{
//			pumpAflag = true;
//		}
//		else if(GPIO_Pin == GPIO_PIN_0 && pumpAflag == true)
//		{
//			pumpAflag = false;
//		}
//	}
//	// Pump B callback
//	if((pumpBcurrent - pumpBprevious) > timeignore)
//	{
//		pumpBprevious = pumpBcurrent;
//		if(GPIO_Pin == GPIO_PIN_2 && pumpBflag == true)
//		{
//			pumpBflag = false;
//		}
//		else if(GPIO_Pin == GPIO_PIN_3 && pumpBflag == false)
//		{
//			pumpBflag = true;
//		}
//	}
//}

void EXTI0_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		pumpAcurrent = HAL_GetTick();
		tms = SysTick->LOAD + 1;
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;              // nano second
		if((pumpAcurrent - pumpAprevious) > timeignore && pumpAflag == true)
		{
			pumpAprevious = pumpAcurrent;
			pumpAflag = false;
		}
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
		pumpAcurrent = HAL_GetTick();
		tms = SysTick->LOAD + 1;
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;
		if((pumpAcurrent - pumpAprevious) > timeignore && pumpAflag == false)
		{
			pumpAprevious = pumpAcurrent;
			pumpAflag = true;
		}
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
		pumpBcurrent = HAL_GetTick();
		tms = SysTick->LOAD + 1;
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;
		if((pumpBcurrent - pumpBprevious) > timeignore && pumpBflag == true)
		{
			pumpBprevious = pumpBcurrent;
			pumpBflag = false;
		}
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
		pumpBcurrent = HAL_GetTick();
		tms = SysTick->LOAD + 1;
		n0 = HAL_GetTick();
		t0 = tms - SysTick->VAL;
		fu0 = n0 * 1000000 + (t0 * 1000000) / tms;  
		if((pumpBcurrent - pumpBprevious) > timeignore && pumpBflag == false)
		{
			pumpBprevious = pumpBcurrent;
			pumpBflag = true;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
		n1 = HAL_GetTick();
		t1 = tms - SysTick->VAL;
		fu1 = n1 * 1000000 + (t1 * 1000000) / tms;
		it = fu1 - fu0;
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
	{
		pumpstop = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	current = HAL_GetTick();
	if(((current - previous) > pulsedelay) && pumpstop == false)        // 1 tick about 1 ms
	{
		previous = current;
		if(pumpAflag == true)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET)
			{
				pumpAcount = pumpAcount + 1;
			}
		}
		if(pumpBflag == true)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET)
			{
				pumpBcount = pumpBcount + 1;
			}
		}
	}
//	if(pumpAcount == pumpdifference)
//	{
//		pumpAcount = 0;
//		pumpAused = pumpAused + 1;
//	}
//	if(pumpBcount == pumpdifference)
//	{
//		pumpBcount = 0;
//		pumpBused = pumpBused + 1;
//	}
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, pumpB_out_Pin|pumpA_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pumpA_off_Pin pumpA_on_Pin pumpB_off_Pin pumpB_on_Pin
                           main_in_Pin */
  GPIO_InitStruct.Pin = pumpA_off_Pin|pumpA_on_Pin|pumpB_off_Pin|pumpB_on_Pin
                          |main_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : pumpB_out_Pin pumpA_out_Pin */
  GPIO_InitStruct.Pin = pumpB_out_Pin|pumpA_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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