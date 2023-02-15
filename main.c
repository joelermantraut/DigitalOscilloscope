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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "lcd16x2.h"
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 		4096
#define USART_TIMEOUT 		1000
#define NS  				128

#define LED_AZUL		GPIO_PIN_15
#define LED_VERDE		GPIO_PIN_12
#define LED_ROJO		GPIO_PIN_14
#define LED_NARANJA		GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_buffer[ADC_BUF_SIZE];

uint8_t buttonFlag = 0;
uint8_t DMA_half_ready = 0;
uint8_t DMA_ready = 0;

uint32_t Wave_LUT_Sin[NS] = {
		2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355,
		3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
		4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
		3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
		2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031,
		944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258, 211, 168, 130, 97, 69, 45, 26, 13,
		4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189, 234, 283, 336, 394, 456, 521, 591, 664, 740,
		820, 902, 987, 1075, 1166, 1258, 1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
};
// Sin wave

uint32_t Wave_LUT_Tri[NS] = {
		0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024, 1088,
		1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856, 1920, 1984, 2048,
		2111, 2175, 2239, 2303, 2367, 2431, 2495, 2559, 2623, 2687, 2751, 2815, 2879, 2943, 3007,
		3071, 3135, 3199, 3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711, 3775, 3839, 3903, 3967,
		4031, 4095, 4031, 3967, 3903, 3839, 3775, 3711, 3647, 3583, 3519, 3455, 3391, 3327, 3263,
		3199, 3135, 3071, 3007, 2943, 2879, 2815, 2751, 2687, 2623, 2559, 2495, 2431, 2367, 2303,
		2239, 2175, 2111, 2048, 1984, 1920, 1856, 1792, 1728, 1664, 1600, 1536, 1472, 1408, 1344,
		1280, 1216, 1152, 1088, 1024, 960, 896, 832, 768, 704, 640, 576, 512, 448, 384, 320, 256,
		192, 128, 64
};
// Triangle wave

uint32_t Wave_LUT_Sq[NS] = {
		4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
		4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
		4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
		4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
		4095, 4095, 4095, 4095, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
// Square wave

uint32_t *Wave_LUT = Wave_LUT_Sin;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, Wave_LUT_Sin, NS, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_buffer, ADC_BUF_SIZE);
  HAL_TIM_Base_Start(&htim3);

  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ADC_buffer, 2 * ADC_BUF_SIZE);
  // Es el doble del buffer porque los valores son de 16 bits, y la USART
  // envia de a 8 bits.

  uint8_t led_state = GPIO_PIN_RESET;
  uint8_t counter = 0;
  uint32_t i = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/*
	 * El LED azul parpadea todo el tiempo, indicando correcto funcionamiento.
	 * El LED naranja se enciende cuando se genera una onda cuadrada.
	 * El LED rojo se enciende cuando se genera una onda senoidal.
	 * El LED verde se enciende cuando se genera una onda triangular.
	 */

    HAL_GPIO_WritePin(GPIOD, LED_AZUL, led_state);
	led_state = !led_state;
	// Indicador de funcionamiento

	if (Wave_LUT == Wave_LUT_Sin) {
		HAL_GPIO_WritePin(GPIOD, LED_NARANJA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_VERDE, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_ROJO, GPIO_PIN_SET);
	} else if (Wave_LUT == Wave_LUT_Tri) {
		HAL_GPIO_WritePin(GPIOD, LED_NARANJA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_VERDE, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LED_ROJO, GPIO_PIN_RESET);
	} else if (Wave_LUT == Wave_LUT_Sq) {
		HAL_GPIO_WritePin(GPIOD, LED_NARANJA, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LED_VERDE, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_ROJO, GPIO_PIN_RESET);
	}
	// Indicador de forma de onda del generador

	if (buttonFlag) {
		counter = (counter + 1) % 3;
		if (!counter) Wave_LUT = Wave_LUT_Sin;
		else if (counter == 1) Wave_LUT = Wave_LUT_Tri;
		else Wave_LUT = Wave_LUT_Sq;

		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, Wave_LUT, NS, DAC_ALIGN_12B_R);

		buttonFlag = 0;
	}

	if (DMA_ready) {
		for(i = 0; i < ADC_BUF_SIZE / 2; i=i+2) {
			ADC_buffer[i] = ADC_buffer[i] | 0x80;
			// Seteo el big mas significativo de la palabra
			// mas significativa
			// Solo de los elementos del canal 1
		}

		DMA_ready = 0;
	}
	if (DMA_half_ready) {
		for(i = ADC_BUF_SIZE / 2; i < ADC_BUF_SIZE; i=i+2) {
			ADC_buffer[i] = ADC_buffer[i] | 0x80;
			// Seteo el big mas significativo de la palabra
			// mas significativa
			// Solo de los elementos del canal 1
		}

		DMA_half_ready = 0;
	}

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		buttonFlag = 1;
	}
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	DMA_half_ready = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	DMA_ready = 1;
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
