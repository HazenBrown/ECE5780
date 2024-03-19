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

//Setting the SADD and NBYTES example
/* Clear the NBYTES and SADD bit fields
* The NBYTES field begins at bit 16, the SADD at bit 0
*/
//I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
/* Set NBYTES = 42 and SADD = 0x14
* Can use hex or decimal values directly as bitmasks.
* Remember that for 7-bit addresses, the lowest SADD bit
* is not used and the mask must be shifted by one.
*/
//I2C2->CR2 |= (42 << 16) | (0x14 << 1);









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
	
	RCC->AHBENR |= (3<<18); //Enable GPIOB and GPIOC
	
	
	//************************************************
	GPIOB->MODER |= (2<<22); 	//Set PB11 to alternate function mode
	GPIOB->OTYPER |= (1<<11); //Set PB11 to open-drain.
	GPIOB->OSPEEDR &= 0;
	GPIOB->PUPDR &= 0;
	
	GPIOB->AFR[1] |= (1 << 12); //Set PB11 to I2C2_SDA
//************************************************	
	GPIOB->MODER |= (2<<26); 	//Set PB13 to alternate function mode
	GPIOB->OTYPER |= (1<<13); //Set PB13 to open-drain.
	GPIOB->OSPEEDR &= 0;
	GPIOB->PUPDR &= 0;
	
	GPIOB->AFR[1] |= (5 << 20); //Set PB13 to I2C2_SCL
//************************************************
	GPIOB->MODER |= (1<<28); 	//Set PB14 to output mode
	GPIOB->OTYPER &= ~(1<<14); //Set PB14 to push-pull type.
	GPIOB->OSPEEDR &= 0;
	GPIOB->PUPDR &= 0;
//************************************************
	GPIOC->MODER |= (1);; 	//Set PC0 to output mode
	GPIOC->OTYPER &= 0; //Set PC0 to push-pull type.
	GPIOC->OSPEEDR &= 0;
	GPIOC->PUPDR &= 0;
//************************************************

//Attempt to set 14 and 0 High
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

//5.3
//Enable I2C2 Peripherial in RCC
RCC->APB1ENR |= (1<<22);

//I2C Timing register configuration for 100kHz
I2C2->TIMINGR |= (1<<28);
I2C2->TIMINGR |= 0x13;
I2C2->TIMINGR |= (0xF<<8);
I2C2->TIMINGR |= (0x2<<16);
I2C2->TIMINGR |= (0x4<<20);
//Enable PE bit i2c2 cr1
I2C2->CR1 |= 1;

//5.4
//Set slave address of 0x69 in SADD[7:1]
I2C2->CR2 |= (0x69 << 1);
//Set number of bytes to transmit to 1
I2C2->CR2 |= (1 << 16);
//Set the write register to 0 to indicate a write operation
I2C2->CR2 &= ~(1 << 10);
//Set the start bit
I2C2->CR2 |= (1 << 13);

//enable LEDs
GPIOC->MODER |= (85<<12);


  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 uint8_t received_data;
	 
  while (1)
  {
    /* USER CODE END WHILE */
		while(!(I2C2->ISR &= (1<<4)) && !(I2C2->ISR &= (1<<1)))
		{
			
		}
		//If NACKF is set...
		if((I2C2->ISR &= (1<<4)))
		{
			//Turn Red LED ON
			GPIOC->BSRR |= (1<<6);
		}
		//If TXIS is set...
		else if(I2C2->ISR &= (1<<1))
		{
			
			//Turn Green LED ON
			
			I2C2->TXDR = 0x0F; //Set the who am I address register
			
			//Wait until the TC flag is set,
			while(!(I2C2->ISR &= (1 << 6)))
			{
				
			}
					
			//Set slave address of 0x69 in SADD[7:1]
			I2C2->CR2 |= (0x69 << 1);
			
			//Set number of bytes to transmit to 1
			I2C2->CR2 |= (1 << 16);
			
			//Set the write register to 1 to indicate a read operation
			I2C2->CR2 |= (1 << 10);
			
			//Set the start bit
			I2C2->CR2 |= (1 << 13);
			
			//Wait for NACK or RXNE is flagged
			while(!(I2C2->ISR &= (1<<2)) && !(I2C2->ISR &= (1<<4)))
			{
			
			}
			
			//If nack is set, turn on Red LED
			if((I2C2->ISR &= (1<<4)))
			{
			//Turn Red LED ON
				GPIOC->BSRR |= (1<<6);
			}
			//If RXNE is set, continue
			else if(I2C2->ISR &= (1<<2))
			{
				
				//Wait until the TC flag is set,
				while(!(I2C2->ISR &= (1 << 6)))
				{
					
				}
				//Read contents of RXDR register
				received_data = I2C2->RXDR;
				
				GPIOC->BSRR |= (1<<7);
				
				//If the contents in the RXDR register match 0xd3 then release the bus
				if(received_data == 0xD3)
				{
					//Turn on green LED
					
					GPIOC->BSRR |= (1<<8);
					
					//set the stop bit
					I2C2->CR2 |= (1<<14);
				}
			}	
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
