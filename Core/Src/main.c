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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <stdio.h>
	#include <string.h>
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

	char DataChar[0xFF];
	volatile uint32_t circle_cnt = 0;
	//volatile uint32_t tmr_value[35] ;
	//volatile uint32_t tmr_value[67] ;
	volatile uint32_t tmr_value[CRC_SIZE+3] ;
	uint8_t	bit[CRC_SIZE+3] = { 0};
	uint32_t suma_bit = 0;

	int offset	=	45;
	uint32_t	offset_counter = 0;
	uint32_t 	offset_res = 45 ;
	uint32_t	offset_array[128] ={45};
	uint32_t 	total_cnt = 0;

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	#define	SOFT_VERSION 100

//  int soft_version_arr_int[3];
//  	soft_version_arr_int[0] = ((SOFT_VERSION) / 1000) %10  ;
//  	soft_version_arr_int[1] = ((SOFT_VERSION) /   10) %100 ;
//  	soft_version_arr_int[2] = ((SOFT_VERSION)       ) %10  ;
//
//  	sprintf(DataChar,"\r\n\r\n\tLoRa-Contact-f103 over sx1278 v%d.%02d.%d \r\n",
//  			soft_version_arr_int[0] ,
//  			soft_version_arr_int[1] ,
//  			soft_version_arr_int[2] ) ;
//  	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
  	#define 	DATE_as_int_str 	(__DATE__)
  	#define 	TIME_as_int_str 	(__TIME__)
  	sprintf(DataChar,"\tBuild: %s. Time: %s.\r\n" ,
  			DATE_as_int_str ,
  			TIME_as_int_str ) ;
  	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
  	HAL_Delay(500);
  	HAL_TIM_Base_Start(&htim3);
  	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);	//HAL_TIM_ACTIVE_CHANNEL_1
  	HAL_TIM_Base_Start(&htim4);

  	uint32_t zero = 0;
  	uint32_t odin = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
					  //uint8_t spi_data[16] = {0};
					 // HAL_SPI_Receive(&hspi1, spi_data, 16, 1000);
	  uint32_t t_min = 300;

  if ( circle_cnt > CRC_SIZE) {
		for (int y=0; y<CRC_SIZE; y++) {
			if 	( tmr_value[y+1] < t_min) {
				t_min = tmr_value[y+1];
			}
		}

		for (int y=0; y<CRC_SIZE; y++) {
			if (tmr_value[y+1] > t_min) {
				bit[y] = 1;
			} else {
				bit[y] = 0;
			}
		}
		circle_cnt = 0;

		suma_bit = 0;
		 for (int k=0; k<CRC_SIZE; k++) {
			 suma_bit = suma_bit + bit[k];
		 }

//		sprintf(DataChar,"   %d", (int)suma_bit);
//		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		 if ( suma_bit > offset_res) {
			 odin++;
		 } else {
			 zero++;
		 }

		 uint32_t promile = (odin*1000) / (odin+zero);
		 if ( promile > 500) {
			 offset++;
		 }
		 if ( promile < 500) {
			 offset--;
		 }

//		 if (offset < 17) {
//			 offset = 17 ;
//		 }
//
//		 if (offset > 25) {
//			 offset = 25;
//		 }

		 if (offset < 34) {
			 offset = 34 ;
		 }

		 if (offset > 52) {
			 offset = 52;
		 }

		offset_array[offset_counter] = offset;
		if (offset_counter < 128) {
			offset_counter++;
		} else {
			offset_counter = 0;
		}

		uint32_t	offset_summa = 0 ;
		for (int i=0; i<128; i++) {
			offset_summa = offset_summa + offset_array[i];
		}
		offset_res = offset_summa/128 ;

		if (offset_counter == 1) {
		  sprintf(DataChar,"\t%03lu %03lu  %02lu\t%d %lu\r\n",  zero, odin, promile, offset, offset_res );
		  HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		}
	} // main if


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
