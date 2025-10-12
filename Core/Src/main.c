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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ws2812b.h>
#include "LiveLed.h"
#include "vt100.h"
#include "display.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _Device_t
{
  uint8_t DO;
  uint8_t DI;

  struct _Diag
  {
    uint32_t LcdTimeout;
    uint32_t UartUnknwonCnt;
    uint32_t UartErrorCnt;
    uint32_t UpTimeSec;
    uint32_t TransactionCnt;
  }Diag;

}Device_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_COUNT   256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
LiveLED_HnadleTypeDef hLiveLed;
Device_t Device;

/*** USB-UART ***/
char    UartRxBuffer[UART_BUFFER_SIZE];
char    UartTxBuffer[UART_BUFFER_SIZE];

/*** WS2812B LED ***/
uint32_t ColorPattern[LED_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/*** Live LED ***/
void LiveLedOn(void);
void LiveLedOff(void);

/*** USB-UART ***/
void UartRxTask(void);
void UartTxTask(void);
char* UartParser(char *line);

/*** Tools ***/
void UpTimeTask(void);

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);
  printf("Hello, World!\r\n");

  /*** Display ***/
  DisplayInit(&hi2c2, SSD1306_I2C_DEV_ADDRESS);
  DisplayClear();
  DisplayUpdate();
  DisplaySetCursor(0, 4);
  DisplayDrawString("Hello World", &GfxFont7x8, SSD1306_WHITE );
  DisplayUpdate();

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  /*** LEDs Strip ***/
  WS2812B_Init(&htim1, &hdma_tim1_ch1, ColorPattern, LED_COUNT);
  WS2812B_SetBrightness(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    static uint32_t timestamp;
    char string[50];


    if(HAL_GetTick() - timestamp > 250)
    {
      timestamp = HAL_GetTick();

      DisplayClear();
      DisplaySetCursor(5, 8);
      sprintf(string,"WS2812B\n"
                     "Uptime:%lu   ",
              Device.Diag.UpTimeSec);

      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE );
      DisplayDrawLine(0, 0, SSD1306_WIDTH - 1, 0, SSD1306_WHITE);
      DisplayDrawLine(0, 0, 0, SSD1306_HEIGHT - 1, SSD1306_WHITE);
      DisplayDrawLine(0, SSD1306_HEIGHT - 1, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, SSD1306_WHITE);
      DisplayDrawLine(SSD1306_WIDTH - 1, 0, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, SSD1306_WHITE);

      DisplayUpdate();
    }

    for(int i = 0; i < LED_COUNT; i+= 3)
    {
      //RGB
      ColorPattern[i ] =  0x0000FF;
      ColorPattern[i + 1] =  0x00FF00;
      ColorPattern[i + 2] =  0xFF0000;
    }


    LiveLedTask(&hLiveLed);

    UartRxTask();
    UartTxTask();
    UpTimeTask();

    WS2812B_Task();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 45;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  memset(UartRxBuffer, 0x00, UART_BUFFER_SIZE);
  if(HAL_UART_Receive_DMA(&huart1, (uint8_t*)UartRxBuffer, UART_BUFFER_SIZE)!= HAL_OK)
    Device.Diag.UartErrorCnt++;
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LIVE_LED_Pin|DIAG_LED_UPDT_CLK_Pin|DIAG_LED_STRING_UPDT_CLK_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : LIVE_LED_Pin DIAG_LED_UPDT_CLK_Pin DIAG_LED_STRING_UPDT_CLK_Pin PA6 */
  GPIO_InitStruct.Pin = LIVE_LED_Pin|DIAG_LED_UPDT_CLK_Pin|DIAG_LED_STRING_UPDT_CLK_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.Diag.UpTimeSec++;
  }
}
/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}
/* printf --------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  //HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

/* UART-----------------------------------------------------------------------*/
void UartRxTask(void)
{
  for(uint8_t i=0; i < UART_BUFFER_SIZE; i++)
  {
    if(UartRxBuffer[i]==UART_TERIMINATION_CHAR)
    {
      HAL_UART_DMAStop(&huart1);
      strcpy(UartTxBuffer, UartParser(UartRxBuffer));
      memset(UartRxBuffer, 0x00, UART_BUFFER_SIZE);
      if(HAL_UART_Receive_DMA(&huart1, (uint8_t*)UartRxBuffer, UART_BUFFER_SIZE)!= HAL_OK)
        Device.Diag.UartErrorCnt++;
      Device.Diag.TransactionCnt++;
    }
  }
}

void UartTxTask(void)
{
  uint8_t txLen = strlen(UartTxBuffer);
  if(txLen != 0)
  {
    UartTxBuffer[txLen] = UART_TERIMINATION_CHAR;
    UartTxBuffer[txLen + 1] = '\0';

    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, txLen + 1, 100);
    UartTxBuffer[0] = 0;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Device.Diag.UartErrorCnt++;
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
}

char* UartParser(char *line)
{
  static char buffer[UART_BUFFER_SIZE];
  char cmd[UART_CMD_LENGTH];
  char arg1[UART_ARG_LENGTH];
  char arg2[UART_ARG_LENGTH];

  memset(buffer, 0x00, UART_BUFFER_SIZE);
  memset(cmd,0x00, UART_CMD_LENGTH);
  memset(arg1,0x00, UART_ARG_LENGTH);
  memset(arg2,0x00, UART_ARG_LENGTH);

  sscanf(line, "%s",cmd);

  if(!strcmp(cmd, "*IDN?"))
  {
    sprintf(buffer, "*IDN? %s", DEVICE_NAME);
  }
  else if(!strcmp(cmd, "*OPC?"))
  {
    strcpy(buffer, "*OPC");
  }
  else if(!strcmp(cmd, "FW?"))
  {
    sprintf(buffer, "%s", DEVICE_FW);
  }
  else if(!strcmp(cmd, "UID?"))
  {
    sprintf(buffer, "%4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
  }
  else if(!strcmp(cmd, "PCB?"))
  {
    sprintf(buffer, "%s", DEVICE_PCB);
  }
  else if(!strcmp(cmd,"UPTIME?"))
  {
     sprintf(buffer, "UPTIME? %08lX", Device.Diag.UpTimeSec);
  }
  else if(!strcmp(cmd,"DI?"))
  {
     sprintf(buffer, "%08hX", Device.DI);
  }
  else if(!strcmp(cmd,"DO?"))
  {
     sprintf(buffer, "%08hX", Device.DO);
  }
  else if(!strcmp(cmd,"UE?"))
  {
    sprintf(buffer, "%08lX", Device.Diag.UartErrorCnt);
  }
  else if(!strcmp(cmd,"DO"))
  {
    sscanf(line, "%s %s",cmd, arg1);
    Device.DO = strtol(arg1, NULL, 16);
    strcpy(buffer, "OK");
  }
  else
  {
    Device.Diag.UartUnknwonCnt++;
  }
  return buffer;
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
#ifdef USE_FULL_ASSERT
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
