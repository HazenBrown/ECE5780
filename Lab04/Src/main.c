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
		//NOTE for putty configuration, Set implicit cr for lf and force echo ON
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

void Parse();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//Global Variables 6
//int readData;
volatile int readData;
volatile int flagSet;
volatile int counter = 0;
volatile char color;
volatile int mode;
	
char errorstring[] = {'E','R','R','O','R',':','\t','I','N','V','A','L','I','D','\t','K','E','Y','\n','\0'}; 
char cmdprompt[] = {'C','M','D','?','\0'};
char testpoint[] = {'a','a','a','\n','\0'};

char char1[] = {'C','H','A','R','1','\n','\0'};
char char2[] = {'C','H','A','R','2','\n','\0'};


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
	
	//Enable RXNE Interrupt
	USART3->CR1 |= (1<<5);
	
	

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	// Part02 Interrupt enable, uncomment for part 2
	/**__NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 0);*/
	//********************************************************
	
	char testchar = 'b';
	char teststring[] = {'a','b','c','\0'};
	

	transtring(cmdprompt);
		
  while (1)
  {
		//Test code from beginning of lab
		//tranchar(testchar);
		//transtring(teststring);	
		//HAL_Delay(200);
		
		//Check and wait on read status flag. ***************
		//Disable interrupt when testing
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
		//****************************************************
		
		//Uncomment for Part02 Code*******************************
		/**if(flagSet == 1){
			
			Parse();
				
			flagSet = 0;
			
		}*/

  }
  /* USER CODE END 3 */
}

void Parse(void){
	
	if(counter == 0){
		color = readData;
		
		if(!(color == 'r'|| color == 'b' || color == 'g' || color == 'o')){
			transtring(errorstring);
			counter = 0;
		}
		else{
			transtring(char1);
			counter++;
		}
	}
	else if(counter == 1){
		mode = readData;
		transtring(char2);
		counter++;
		
	}
	
	//transtring(testpoint);
	if(counter == 2){
		if(color == 'r'){
			if(mode == '0'){
				//Turn off LED
				GPIOC->BSRR |= (1<<22);
			}
			else if(mode == '1'){
				//Turn on LED
				GPIOC->BSRR |= (1<<6);
			}
			else if(mode == '2'){
				//Toggle LED
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			}
			else
				transtring(errorstring);
			
		}
		else if(color == 'o'){
			if(mode == '0'){
				//Turn off LED
				GPIOC->BSRR |= (1<<24);
			}
			else if(mode == '1'){
				//Turn on LED
				GPIOC->BSRR |= (1<<8);
			}
			else if(mode == '2'){
				//Toggle LED
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			}
			else
				transtring(errorstring);
			
			
		}
		else if(color == 'g'){
			if(mode == '0'){
				//Turn off LED
				GPIOC->BSRR |= (1<<25);
			}
			else if(mode == '1'){
				//Turn on LED
				GPIOC->BSRR |= (1<<9);
			}
			else if(mode == '2'){
				//Toggle LED
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			}
			else
				transtring(errorstring);
				
			
		}
		else if(color == 'b'){
			if(mode == '0'){
				//Turn off LED
				GPIOC->BSRR |= (1<<23);
			}
			else if(mode == '1'){
				//Turn on LED
				GPIOC->BSRR |= (1<<7);
			}
			else if(mode == '2'){
				//Toggle LED
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
			}
			else
				transtring(errorstring);
		}
		//Error message if invalid color is entered
		else if(color != 13 || color != 'r' || color != 'g' || color != 'b' || color != 'o')  {
			transtring(errorstring);
		}
		counter = 0;
	}
		
	
}
	
//USART3 Interrupt handler
void USART3_4_IRQHandler(void){
	
	if (USART3->ISR & USART_ISR_RXNE){
	//transtring(testpoint);
		flagSet = 1;
		
		readData = USART3->RDR;
		//transtring(testpoint);
	}
	
}
// Function used to transmit a character
// Takes a single character parameter and saves that into the TDR register.
void tranchar(char key)
{
	//While the transmit data register is not empty...
	// USART3->ISR != (USART3->ISR &= USART_ISR_TXE_Msk) 
	while(!(USART3->ISR & USART_ISR_TXE))
	{
		
	}
	
	USART3->TDR = key;
}

//Function used to transmit a string of chars
//Takes an array of chars and transmits each char seperately. 
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
