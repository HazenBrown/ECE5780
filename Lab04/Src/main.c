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

void tranchar(char key);

void transtring(char key[]);
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
	
	
	//*******************************************************
	
	//Configure LEDs and enable hardware
	RCC->AHBENR |= (1<<19); //Enable the 19th bit for clock
	
	GPIOC->MODER |= (85<<12);
	GPIOC->OTYPER &= 0;
	GPIOC->OSPEEDR &= 0;
	GPIOC->PUPDR &= 0;
	
	//Enable alternate function mode on PC4 and PC5
	GPIOC->MODER |= (10 << 8);
	
	
	//Enable afrl for pc4 and pc5 
	// 00010001 shifted left 16 (AF1) (AF1)
	GPIOC->AFR[0] |= (17 << 16); 
	
	//Enable the clock to the USART register. 
	//Writes a 1 to enable USART3 Clock
	RCC->APB1ENR |= (1 << 18);
	
	//Set the baud rate BRR value
	USART3->BRR = 69; //69(HAL_RCC_GetHCLKFreq()/115200)
	
	//Enable transmit and recieve as well as enable the USART
	USART3->CR1 |= 13;
	
	
	//********************************************************
	

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	char testchar = 'b';
	char teststring[] = {'a','b','c','\0'};
	char errorstring[] = {'E','R','R','O','R',':','\t','I','N','V','A','L','I','D','\t','K','E','Y','\t'}; 
	
  while (1)
  {
		//Test code from beginning of lab
		//tranchar(testchar);
		//transtring(teststring);	
		//HAL_Delay(200);
		
		//Check and wait on read status flag.
		while ((USART3->ISR & USART_ISR_RXNE) == 0) {
    
    }
		
		//Read register contents into a temporary variable
		char regread = USART3->RDR;
	
		//Check read character and toggle LEDs respectively
		
		if(regread == 'r'){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		}
		else if(regread == 'o'){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		}
		else if(regread == 'g'){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		}
		else if(regread == 'b'){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		}
		else if(regread != 13 || regread != 'r' || regread != 'g' || regread != 'b' || regread != 'o')  {
			transtring(errorstring);
		}
		
		
		
  }
  /* USER CODE END 3 */
}

void tranchar(char key)
{
	//While the transmit data register is not empty...
	// USART3->ISR != (USART3->ISR &= USART_ISR_TXE_Msk) 
	while(!(USART3->ISR & USART_ISR_TXE))
	{
		
	}
	
	USART3->TDR = key;
}

void transtring(char key[])
{
	
		char current = key[0];
		
		for(int i = 0; current != 0; i++)
		{
			current = key[i];
			tranchar(current);
		}
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
