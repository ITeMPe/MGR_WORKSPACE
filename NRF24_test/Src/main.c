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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nRF24L01.h"
#include "MY_NRF24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_MODE
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
#ifndef RX_MODE
uint64_t TxpipeAddrs = 0xF0F0F0F0D2;
char myTxData[32] = "Hello World form my STM32F429ZI!";
char AckPayload[32];
#else
uint64_t RxpipeAddrs = 0xF0F0F0F0E1;
char myRxData[50];
char myAckPayload[32] = "Ack by STMF4!";
#endif
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
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin|LED2_Pin|LED3_Pin,SET);
	NRF24_begin(CSN_GPIO_Port, CSN_Pin, CE_Pin, hspi3);
	nrf24_DebugUART_Init(huart2);

#ifndef RX_MODE
	//**** TRANSMIT - ACK ****//
//	NRF24_openReadingPipe(1,ADRESNADAWCZY);
	NRF24_openWritingPipe(TxpipeAddrs);
	NRF24_setPALevel(RF24_PA_0dB);
	NRF24_setDataRate(RF24_250KBPS);
	NRF24_setCRCLength(RF24_CRC_16);
	NRF24_setRetries(15, 15);
	NRF24_setChannel(120);

	NRF24_setPayloadSize(32);
//	NRF24_setAutoAck(false);

	NRF24_stopListening();

//	NRF24_enableDynamicPayloads();
//	NRF24_enableAckPayload();
#else

	NRF24_setChannel(125);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(0, RxpipeAddrs);
	NRF24_setPALevel(RF24_PA_0dB);
	NRF24_setRetries(15, 15);
	NRF24_setDataRate(RF24_250KBPS);
	NRF24_setCRCLength(RF24_CRC_16);

	NRF24_startListening();

#endif

printRadioSettings();

//uint8_t datatx[] =  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
//uint8_t datarx;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifndef RX_MODE
	if (NRF24_write(myTxData, 32)) {
//		NRF24_read(AckPayload, 32);
		HAL_UART_Transmit(&huart2,
				(uint8_t *) "Transmitted Successfully\r\n",
				strlen("Transmitted Successfully\r\n"), 10);
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED2_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED2_Pin);
		HAL_Delay(50);
//		char myDataack[80];
//		sprintf(myDataack, "AckPayload:  %s \r\n", AckPayload);
//		HAL_UART_Transmit(&huart2, (uint8_t *) myDataack, strlen(myDataack),
//				10);
	}
	HAL_Delay(1000);
#else
	if(NRF24_available())
	{
		NRF24_read(myRxData, 32);
		NRF24_writeAckPayload(1, myAckPayload, 32);
		myRxData[32] = '\r'; myRxData[32+1] = '\n';
		HAL_UART_Transmit(&huart2, (uint8_t *)myRxData, 32+2, 10);
	}
#endif
/*
 * only for test interface
 */
//	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,RESET);
//	  HAL_SPI_Transmit(&hspi1,(uint8_t*)datatx,sizeof(datatx),10);
//	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,SET);
//	  for(uint8_t i=0x0;i<16;i++)
//	  {
//
//	  	  HAL_GPIO_WritePin(nrf_CSN_PORT,nrf_CSN_PIN,RESET);
//	  	  HAL_SPI_Transmit(&hspi3, &i,1,10);
//	  	  HAL_GPIO_WritePin(nrf_CSN_PORT,nrf_CSN_PIN,SET);
//	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
