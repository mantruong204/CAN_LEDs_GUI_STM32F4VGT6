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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
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
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t TxData[2];
uint8_t RxData[2];
uint8_t count =0;

int MAX_LEN=10;
uint8_t nRxData[10];
uint8_t nTxData[10];
uint8_t nTxData_CAN[10];
uint8_t strCommand[4];
uint8_t strOpt[1];
uint8_t strData[2];

bool bDataAvailable = false;
uint8_t STX[]={0x02U};
uint8_t ETX[]={0x03U};
uint8_t ACK[]={0x06U};
uint8_t SYN[]={0x16U};
uint8_t NUL[]={0x4EU, 0x55U, 0x4CU, 0x4CU};
uint8_t ONLD[]={0x4FU, 0x4EU, 0x4CU, 0x44U};
uint8_t VADC[]={0x56U, 0x41U, 0x44U, 0x43U};
uint8_t RBTN[]={0x52U, 0x42U, 0x54U, 0x4EU};
//uint8_t OFLD[]={0x4FU, 0x46U, 0x4CU, 0x44U};

uint16_t CanRxADC_Val = 0;

uint8_t *subString(uint8_t *s, int pos, int index)
{
	uint8_t *t =&s[pos];
	/*s[pos-1] = '\0';
	for (int i = index; i < (strlen((char *)t)+1); i++)
	{
		t[i]='\0';
	}*/
	return t;
}


bool StrCompare(uint8_t *pBuff, uint8_t *Sample, uint8_t nSize)
{
	for (int i=0; i< nSize; i++)
	{
		if (pBuff[i] != Sample[i])
		{
			return false;
		}
	}
	return true;
}


bool WriteComm(uint8_t *pBuff, uint8_t nSize)
{
	return HAL_UART_Transmit_DMA(&huart2, pBuff, nSize);
}

bool ReadComm(uint8_t *pBuff, uint8_t nSize)
{
	if ((pBuff[0] == STX[0]) && (pBuff[9] == ETX[0]))
	{
		memcpy(strCommand, subString(pBuff, 1, 4), 4);
		memcpy(strOpt, subString(pBuff, 5, 1), 1);
		memcpy(strData, subString(pBuff, 6, 2), 2);
		
		bDataAvailable = true;
	}
	else{
		bDataAvailable = false;
	}
	return bDataAvailable;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	uint8_t nIndex_CAN =0;

	if(RxHeader.StdId == 0x0B)
	{
		CanRxADC_Val = (uint16_t)((RxData[0]<<8) + RxData[1]);

		uint8_t ID_valB=0x0B;
		memcpy(nTxData_CAN + nIndex_CAN, STX, 1);
		nIndex_CAN +=1;
		memcpy(nTxData_CAN + nIndex_CAN, VADC, 4);
		nIndex_CAN +=4;
		memcpy(nTxData_CAN + nIndex_CAN, &ID_valB, 1);
		nIndex_CAN +=1;
		memcpy(nTxData_CAN + nIndex_CAN, RxData, 2);
		nIndex_CAN +=2;
		memcpy(nTxData_CAN + nIndex_CAN, ACK, 1);
		nIndex_CAN +=1;
		memcpy(nTxData_CAN + nIndex_CAN, ETX, 1);
		nIndex_CAN +=1;
		WriteComm(nTxData_CAN, MAX_LEN);
	}
	else if (RxHeader.StdId == 0x0C)
	{
		uint8_t ID_valC =0x0C;
		memcpy(nTxData_CAN + nIndex_CAN, STX, 1);
		nIndex_CAN +=1;
		memcpy(nTxData_CAN + nIndex_CAN, RBTN, 4);
		nIndex_CAN +=4;
		memcpy(nTxData_CAN + nIndex_CAN, &ID_valC, 1);
		nIndex_CAN +=1;
		memcpy(nTxData_CAN + nIndex_CAN, RxData, 2);
		nIndex_CAN +=2;
		memcpy(nTxData_CAN + nIndex_CAN, ACK, 1);
		nIndex_CAN +=1;
		memcpy(nTxData_CAN + nIndex_CAN, ETX, 1);
		nIndex_CAN +=1;
		WriteComm(nTxData_CAN, MAX_LEN);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		ReadComm(nRxData, MAX_LEN);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)nRxData, MAX_LEN);
	}
}

bool serialProcess(void)
{
	uint8_t nIndex =0;
	if (bDataAvailable == true)
	{
		if(StrCompare(strCommand, ONLD, 4))
		{
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
			memcpy(nTxData + nIndex, STX, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, strCommand, 4);
			nIndex +=4;
			memcpy(nTxData + nIndex, strOpt, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, strData, 2);
			nIndex +=2;
			memcpy(nTxData + nIndex, ACK, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, ETX, 1);
			nIndex +=1;
			WriteComm(nTxData, MAX_LEN);

			TxHeader.DLC = 2;
			TxHeader.ExtId = 0;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.StdId = 0x0A;
			TxHeader.TransmitGlobalTime = DISABLE;

			TxData[0] = nTxData[6]; // Data send to CAN BUS to turn on led with rule
			TxData[1] = nTxData[7];

			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData,&TxMailbox) != HAL_OK)
				{
					NVIC_SystemReset();
				}
		}
		/*if(StrCompare(strCommand, VADC, 4))
		{
			memcpy(nTxData + nIndex, STX, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, strCommand, 4);
			nIndex +=4;
			memcpy(nTxData + nIndex, strOpt, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, strData, 2);
			nIndex +=2;
			memcpy(nTxData + nIndex, ACK, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, ETX, 1);
			nIndex +=1;
			//WriteComm(nTxData, MAX_LEN);

			TxHeader.DLC = 2;
			TxHeader.ExtId = 0;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.StdId = 0x0B;
			TxHeader.TransmitGlobalTime = DISABLE;

			TxData[0] = nTxData[6]; // Data send to CAN BUS to turn on led with rule
			TxData[1] = nTxData[7];
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData,&TxMailbox);
		}
		if(StrCompare(strCommand, RBTN, 4))
		{
			memcpy(nTxData + nIndex, STX, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, strCommand, 4);
			nIndex +=4;
			memcpy(nTxData + nIndex, strOpt, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, strData, 2);
			nIndex +=2;
			memcpy(nTxData + nIndex, ACK, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, ETX, 1);
			nIndex +=1;
			//WriteComm(nTxData, MAX_LEN);

			TxHeader.DLC = 2;
			TxHeader.ExtId = 0;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.StdId = 0x0C;
			TxHeader.TransmitGlobalTime = DISABLE;

			TxData[0] = nTxData[6]; // Data send to CAN BUS to turn on led with rule
			TxData[1] = nTxData[7];
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData,&TxMailbox);
		}*/
else
		{
			memcpy(nTxData + nIndex, STX, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, NUL, 4);
			nIndex +=4;
			memcpy(nTxData + nIndex, (uint8_t *)strOpt, 1);
			nIndex +=3;
			memcpy(nTxData + nIndex, (uint8_t *)strData, 2);
			nIndex +=8;
			memcpy(nTxData + nIndex, ACK, 1);
			nIndex +=1;
			memcpy(nTxData + nIndex, ETX, 1);
			nIndex +=1;
			WriteComm(nTxData, MAX_LEN);
		}	

bDataAvailable = false;
	}
	return true;
}
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)nRxData, MAX_LEN);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_IWDG_Refresh (&hiwdg);
		serialProcess();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x000<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x000<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 1499;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
