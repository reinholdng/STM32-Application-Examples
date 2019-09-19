/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ringbuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "Welcome to the Nucleo management console\r\n"
#define MAIN_MENU   "Select the option you are interested in:\r\n\t1. Toggle LD2 LED\r\n\t2. Read USER BUTTON status\r\n\t3. Clear screen and print this message "
#define PROMPT "\r\n> "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int8_t cntcmd = 0;
int8_t cntcmd2 = 0;
int8_t counter =0;
int8_t uartrxcnt=0;
char cnt[10];
//int8_t cmdcompleto = 0;
char command[64];
char command2[64];
char readBuf[1];
char readBuf2[1];
char newline[2] = "\r\n";
uint8_t txData;
__IO ITStatus UartReadyTx2 = SET;
__IO ITStatus UartReadyRx2 = SET;
__IO ITStatus UartReadyTx3 = SET;
__IO ITStatus UartReadyRx3 = SET;
RingBuffer txBuf, rxBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void printWelcomeMessage(void);
int8_t readUserInput(void);
int8_t readLoraInput(void);
uint8_t processUserInput(int8_t opt);
uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len);
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

	uint8_t opt = 0;
	uint8_t opt2 = 0;
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(USART2_IRQn);
  //HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(USART3_IRQn);


  //printMessage:
  	  printWelcomeMessage();
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
  	  HAL_Delay(10000);
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	  HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //opt = readUserInput();

	  
	  if(UartReadyRx2 == SET){
		UartReadyRx2 = RESET;

		if(readBuf[0]=='\r'){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			command[cntcmd]=readBuf[0];
			cntcmd++;
			command[cntcmd]='\n';
			readBuf[0]=0;

			//UART_Transmit(&huart1, (uint8_t*)command, cntcmd);
			//while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
			//UART_Transmit(&huart1, (uint8_t*)newline, strlen(newline));
			//while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);

			//HAL_UART_Transmit(&huart2, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart3, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
			UART_Transmit(&huart3, (uint8_t*)command, cntcmd);
			while (HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX_RX);
			//UART_Transmit(&huart3, (uint8_t*)newline, strlen(newline));
			//while (HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX_RX);
			//Enter Reception Mode For UART3

			HAL_UART_Receive_IT(&huart3, (uint8_t*)readBuf2, 1);
			cntcmd = 0;
			for (int8_t i = 0; i < sizeof(command)/sizeof(command[0]); i++){
				command[i] = 0;
			}
		}else {
			command[cntcmd]=readBuf[0];
			cntcmd++;
			readBuf[0]=0;
			HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 1);
		}

		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  }

	if(UartReadyRx3 == SET){
		UartReadyRx3 = RESET;
		if(readBuf2[0]== '#'){
			command2[cntcmd2]=readBuf2[0];
			cntcmd2++;
			command2[cntcmd2]='\n';
			readBuf2[0]=0;
			//uartrxcnt++;

			UART_Transmit(&huart1, (uint8_t*)command2, cntcmd2);
			while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
			//UART_Transmit(&huart1, (uint8_t*)newline, strlen(newline));
			//while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
			//RETURN TO RX MODE FOR UART2

			//if(uartrxcnt==2){
			HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 1);
			//	uartrxcnt=0;
			//}else{
			//HAL_UART_Receive_IT(&huart3, (uint8_t*)readBuf, 1);
			//}
			cntcmd2 = 0;
			for (int8_t i = 0; i < sizeof(command2)/sizeof(command2[0]); i++){
				command2[i] = 0;
			}
		}else {
			command2[cntcmd2]=readBuf2[0];
			cntcmd2++;
			readBuf2[0]=0;
			HAL_UART_Receive_IT(&huart3, (uint8_t*)readBuf2, 1);
		}

		  //counter++;
		  //itoa(counter,cnt,10);
		  //UART_Transmit(&huart2, (uint8_t*)cnt, strlen(cnt));
		  //while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX_RX);
		  //UART_Transmit(&huart2, (uint8_t*)newline, strlen(newline));
		  //while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX_RX);
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len) {
  if(HAL_UART_Transmit_IT(huart, pData, len) != HAL_OK) {
    if(RingBuffer_Write(&txBuf, pData, len) != RING_BUFFER_OK)
      return 0;
  }
  return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

	if(UartHandle->Instance==USART1)
	 {
		UartReadyRx2 = SET;

	 }else if(UartHandle->Instance==USART3)
	 {
		UartReadyRx3 = SET;
	 }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

		if(RingBuffer_GetDataLength(&txBuf) > 0) {
			RingBuffer_Read(&txBuf, &txData, 1);
			HAL_UART_Transmit_IT(huart, &txData, 1);
		}


}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if(huart->ErrorCode == HAL_UART_ERROR_ORE)
    HAL_UART_Receive_IT(huart, (uint8_t *)readBuf, 1);
}

void printWelcomeMessage(void) {
	char *strings[] = {"\033[0;0H", "\033[2J", WELCOME_MSG, MAIN_MENU, PROMPT};

	for (uint8_t i = 0; i < 5; i++) {
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)strings[i], strlen(strings[i]));
		while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
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

	while(1)
	  {
	    /* Toggle LED2 for error */
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	    HAL_Delay(500);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
