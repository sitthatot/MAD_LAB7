/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define print(x) HAL_UART_Transmit(&huart3, (uint8_t*)x, strlen(x),1000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t pwm;
float dutyCycleR = 0.0;
float dutyCycleG = 0.0;
float dutyCycleB = 0.0;
uint8_t countR = 0;
uint8_t countG = 0;
uint8_t countB = 0;
char toString[100];

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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); //Blue
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); //Red
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); //Green
  htim2.Instance -> CCR4 = (10000-1) * dutyCycleB;//Blue PWM
  htim3.Instance -> CCR3 = (10000-1) * dutyCycleR;//Red PWM
  htim4.Instance -> CCR3 = (10000-1) * dutyCycleG;//Green PWM


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  char buffer;
	  print("input => ");
	  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)==RESET){}//check buffer empty?
	  HAL_UART_Receive(&huart3, (uint8_t*)&buffer, 1,1000);
	  print(&buffer);
	  print("\r\n");




	  if(buffer == 'B' || buffer == 'b')
	  {

		  if(countB < 5)
		  {
			  print("This is BBBBBBBBBBB");
			  dutyCycleB += 0.20;//Blue PWM
			  print(toString);
			  htim2.Instance -> CCR4 = (10000-1) * dutyCycleB;//Blue PWM
			  countB++;
		  }
		  else
		  {
			  countB = 0;
			  dutyCycleB = 0.0;
			  htim2.Instance -> CCR4 = (10000-1) * dutyCycleB;//Blue PWM
		  }
	  }
	  else if(buffer == 'R' || buffer == 'r')
	  {

		  if(countR < 5)
		  		  {
			  print("This is RRRRRRRRRRRRR");
		  			  dutyCycleR += 0.20;//Red PWM
		  			htim3.Instance -> CCR3 = (10000-1) * dutyCycleR;//Red PWM
		  			countR++;
		  		  }
		  		  else
		  		  {
		  			  countR = 0;
		  			dutyCycleR = 0.0;
		  			htim3.Instance -> CCR3 = (10000-1) * dutyCycleR;//Red PWM
		  		  }

	  }
	  else if(buffer == 'G' || buffer == 'g')
	  {
	 	 if(countG < 5)
	 	 		  {
	 		print("This is GGGGGGGGGG");
	 	 			  dutyCycleG += 0.20;//Green PWM
	 	 			htim4.Instance -> CCR3 = (10000-1) * dutyCycleG;//Green PWM
	 	 			countG++;

	 	 		  }
	 	 		  else
	 	 		  {
	 	 			  countG = 0;
	 	 			dutyCycleG = 0.0;
	 	 			htim4.Instance -> CCR3 = (10000-1) * dutyCycleG;//Green PWM
	 	 		  }
	  }

	  HAL_Delay(100);

	  pwm=(GPIOB->IDR & GPIO_PIN_15)>>10;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
