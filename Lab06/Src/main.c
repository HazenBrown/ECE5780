/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
// Triangle Wave: 8-bit, 32 samples/cycle
//const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
//190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

	RCC->AHBENR |= (7<<17); //Enable GPIOB and GPIOC and GPIOA
	
	//enable LEDs
	GPIOC->MODER |= (85<<12); 
	GPIOC->OTYPER &= 0;
	GPIOC->OSPEEDR &= 0;
	GPIOC->PUPDR &= 0;
	
	GPIOC->MODER |= 3; //PC0 configure to analog mode.
	__HAL_RCC_ADC1_CLK_ENABLE();
	//__HAL_RCC_DAC1_CLK_ENABLE();
	
	ADC1->CFGR1 |= (1<<13);

	ADC1->CFGR1 |= (1<<4);	//Set 8 bit resolution
	ADC1->CFGR1 &= ~(1<<3);	//Set 8 bit resolution
	
	ADC1->CFGR1 &= ~(1<<11); //Triggers?
	ADC1->CFGR1 &= ~(1<<10);
	
	ADC1->CHSELR |= (1<<10); //Set ADC input (PC0 is 10)
	
	////////////////////////////////////////////////////////////
	/**
	//Part02
	//DACout1 PA4
	GPIOA->MODER |= (3<<8); //PA4 set to analog mode
	GPIOA->OTYPER &= 0;
	GPIOA->OSPEEDR &= 0;
	GPIOA->PUPDR &= 0;
	
	DAC1->CR |= (1<<3); // Configure software trigger
	DAC1->CR |= (1<<4); 
	DAC1->CR |= (1<<5);
	
	DAC1->CR |= (1<<0);// Enable dac channel 1
	
	*/
	////////////////////////////////////////////////////////////
	
	
	
	//calibrate ADC
	ADC1->CR |= (1<<31); //Set ADCAL to 1;
	
	//Wait until bit is 0
	while(ADC1->CR &= (1<<31)){
		
	}
	
	//Enable ADC
	
	
	ADC1->CR |= (1<<0);
	ADC1->CR |= (1<<2);
	ADC1->ISR |= (1<<0);

	
	
	
	uint8_t readval;
	
	uint8_t burn = 0;
	
  while (1)
  {
		
    /* USER CODE END WHILE */
		
		/**
		DAC1->DHR8R1 = triangle_table[burn];
		
		
		
		burn++;
		
		
		
		if(burn == 31){
		burn = 0;
		}
		
		HAL_Delay(1);
		*/
		
		
		//Part01
		//Analog read
		
		readval = ADC1->DR;
		
		if(readval > 50){
			//Turn on Blue LED
			GPIOC->BSRR |= (1<<7);
		
		}
		if(readval > 100){
			//Turn on orange LED
			GPIOC->BSRR |= (1<<8);
		
		}
		if(readval > 150){
			//Turn on Red LED
			GPIOC->BSRR |= (1<<6);
		
			
		}
		if(readval > 200){
			//Turn on Green LED
			GPIOC->BSRR |= (1<<9);
		}
		else{
			GPIOC->BSRR |= (1<<25);
			GPIOC->BSRR |= (1<<24);
			GPIOC->BSRR |= (1<<22);
			GPIOC->BSRR |= (1<<23);
		}
		
		
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
