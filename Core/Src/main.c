/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define m1_p CCR1
#define m2_p CCR2
#define m3_p CCR3
#define m4_p CCR4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t data[20];
uint8_t uart_parse_flag = 0;
uint8_t uart_size;
char data_to_bt[20];
int x, y;
float distance;
uint8_t uart_disable_flag = 0;
uint16_t adc[2];
float battery_voltage=0,temp=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
inline void forward()
{
	HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 0);
	HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 0);
	HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 0);
	HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 0);
	HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
	HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
	HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
	HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
}
inline void backwards()
{
	HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 0);
	HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 0);
	HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 0);
	HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 0);
	HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
	HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
	HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
	HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
}
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	TIM1->m1_p = 400;
	TIM1->m2_p = 400;
	TIM1->m3_p = 300;
	TIM1->m4_p = 400;
	HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 0);
	HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 0);
	HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 0);
	HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 0);
	HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
	HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
	HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
	HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data, 20);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(battery_voltage>6.5)
		{
			if (uart_disable_flag)
			{

				if (distance < 50)
				{
					HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
					HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
					HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
					HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
					HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
					HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
					HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
					HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
					TIM1->CCR1 = 9999;
					TIM1->CCR2 = 9999;
					TIM1->CCR3 = 9999;
					TIM1->CCR4 = 9999;
					HAL_Delay(10);
					TIM1->CCR1 = 4000;
					TIM1->CCR2 = 4000;
					TIM1->CCR3 = 4000;
					TIM1->CCR4 = 4000;
					HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 0);
					HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 0);
					HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 0);
					HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 0);
					HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
					HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
					HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
					HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
					HAL_Delay(500);
					HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
					HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
					HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 0);
					HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 0);
					HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 0);
					HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 0);
					HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
					HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);

					HAL_Delay(500);

				}
				else
				{
					HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
					HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
					HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
					HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
					HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 0);
					HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 0);
					HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 0);
					HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 0);
					TIM1->CCR1 = 5000;
					TIM1->CCR2 = 5000;
					TIM1->CCR3 = 5000;
					TIM1->CCR4 = 5000;
				}
			}
			else
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				if (uart_parse_flag)
				{
					if (data[0] == 'X')
					{
						x = atoi((char*) data + 2);
					}
					else if (data[0] == 'Y')
					{
						y = atoi((char*) data + 2);
					}
					if (x > 3000)
					{
						HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
						HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
						HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
						HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
						HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 0);
						HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 0);
						HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 0);
						HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 0);
						if (y > 0)
						{
							if (0 >= (x - y))
							{
								TIM1->m1_p = 0;
								TIM1->m2_p = 0;
								TIM1->m3_p = (x + y);
								TIM1->m4_p = (x + y);
							}
							else
							{
								TIM1->m1_p = x - y;
								TIM1->m2_p = x - y;
								TIM1->m3_p = (x + y);
								TIM1->m4_p = (x + y);
							}

						}
						else
						{
							if ((x + y) <= 0)
							{
								TIM1->m1_p = x - y;
								TIM1->m2_p = x - y;
								TIM1->m3_p = 0;
								TIM1->m4_p = 0;
							}
							else
							{
								TIM1->m1_p = x - y;
								TIM1->m2_p = x - y;
								TIM1->m3_p = x + y;
								TIM1->m4_p = x + y;
							}

						}
					}
					else if (x < -3000)
					{
						HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 0);
						HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 0);
						HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 0);
						HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 0);
						HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
						HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
						HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
						HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
						if (y > 0)
						{
							if ((-x - y) <= 0)
							{
								TIM1->m3_p = -x;
								TIM1->m4_p = -x;
								TIM1->m1_p = 0;
								TIM1->m2_p = 0;
							}
							else
							{
								TIM1->m3_p = -x;
								TIM1->m4_p = -x;
								TIM1->m1_p = -x - y;
								TIM1->m2_p = -x - y;
							}
						}
						else
						{

							if ((-x + y) <= 0)
							{
								TIM1->m3_p = 0;
								TIM1->m4_p = 0;
								TIM1->m1_p = (-x + y);
								TIM1->m2_p = (-x + y);
							}
							else
							{
								TIM1->m3_p = -x - y;
								TIM1->m4_p = -x - y;
								TIM1->m1_p = (-x + y);
								TIM1->m2_p = (-x + y);
							}

						}
					}
					else if (y > 3000)
					{
						TIM1->CCR1=0;
						TIM1->CCR2=0;
						TIM1->CCR3=y;
						TIM1->CCR4=y;
						HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
						HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
						HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
						HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
						HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
						HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
						HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 0);
						HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 0);
					}
					else if (y < -3000)
					{
						TIM1->CCR1=(-y);
						TIM1->CCR2=(-y);
						TIM1->CCR3=0;
						TIM1->CCR4=0;
						HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
						HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
						HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
						HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
						HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 0);
						HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 0);
						HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
						HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
					}
					else
					{
						TIM1->CCR1=0;
						TIM1->CCR2=0;
						TIM1->CCR3=0;
						TIM1->CCR4=0;
						HAL_GPIO_WritePin(m1_1_GPIO_Port, m1_1_Pin, 1);
						HAL_GPIO_WritePin(m2_1_GPIO_Port, m2_1_Pin, 1);
						HAL_GPIO_WritePin(m3_1_GPIO_Port, m3_1_Pin, 1);
						HAL_GPIO_WritePin(m4_1_GPIO_Port, m4_1_Pin, 1);
						HAL_GPIO_WritePin(m1_2_GPIO_Port, m1_2_Pin, 1);
						HAL_GPIO_WritePin(m2_2_GPIO_Port, m2_2_Pin, 1);
						HAL_GPIO_WritePin(m3_2_GPIO_Port, m3_2_Pin, 1);
						HAL_GPIO_WritePin(m4_2_GPIO_Port, m4_2_Pin, 1);
					}
				}
			}
		}
		else
		{
			TIM1->CCR1=0;
			TIM1->CCR2=0;
			TIM1->CCR3=0;
			TIM1->CCR4=0;
			while(1){HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);}
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data, 20);
	uart_parse_flag = 1;
	uart_size = Size;
	TIM2->CNT = 0;
	uart_disable_flag = 0;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
			float micro = ((TIM4->CCR2) - (TIM4->CCR1));
			distance = (micro / 58.0);
		}
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	battery_voltage=adc[0]*3.300/4095.000/0.233;
	temp=adc[1]*330/4095;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		uart_disable_flag = 1;
	}
	else if(htim->Instance==TIM4)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 2);
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

