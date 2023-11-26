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
#include "LiveLed.h"
#include "vt100.h"
#include "SSD1306.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _Devic_t
{
  uint8_t DO;
  uint8_t DI;

  struct _Diag
  {
    uint32_t LcdTimeout;
    uint32_t UsbUartUnknwonCnt;
    uint32_t UsbUartErrorCnt;
    uint32_t UpTimeSec;
  }Diag;

}Device_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USB_UART_BUFFER_SIZE    64
#define USB_UART_CMD_LENGTH     35
#define USB_UART_ARG_LENGTH     35

#define STRIP_LEDS_COUNT      10
#define STRIP_COLORS_PER_LED  3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LiveLED_HnadleTypeDef hLiveLed;
Device_t Device;


/*** USB-UART ***/
char    UsbUartRxBuffer[USB_UART_BUFFER_SIZE];
char    UsbUartTxBuffer[USB_UART_BUFFER_SIZE];
__IO char    UsbUartCharacter;
__IO uint8_t UsbUartRxBufferPtr;

//uint32_t DmaBuffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/*** Live LED ***/
void LiveLedOn(void);
void LiveLedOff(void);

/*** USB-UART ***/
char* UsbUartParser(char *line);
void UsbUartTxTask(void);

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
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

  /*** Display ***/
  SSD1306_Init(&hi2c2, SSD1306_I2C_DEV_ADDRESS);
  SSD1306_DisplayClear();
  SSD1306_DisplayUpdate();

  SSD1306_DrawPixel(0, 0, SSD1306_WHITE);
  SSD1306_DrawPixel(SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, SSD1306_WHITE);
  SSD1306_DrawLine(0, 1, 127, 1, SSD1306_WHITE);
  SSD1306_DrawLine(0, 2, 127, 2, SSD1306_WHITE);
  SSD1306_DrawLine(3, 3, 4, 4, SSD1306_WHITE);

  SSD1306_SetCursor(0, 4);
  SSD1306_DrawString("Hello World", &GfxFont7x8, SSD1306_WHITE );
  SSD1306_DisplayUpdate();

  //SSD1306_DisplayClear();
  //SSD1306_DisplayUpdate();

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);
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
      SSD1306_SetCursor(1, 1);
      sprintf(string,"Uptime:%lu", Device.Diag.UpTimeSec);
      SSD1306_DisplayClear();
      SSD1306_DrawString(string, &GfxFont7x8, SSD1306_WHITE );
      SSD1306_DisplayUpdate();
    }

    LiveLedTask(&hLiveLed);
    UpTimeTask();
    UsbUartTxTask();

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)&UsbUartCharacter, 1) != HAL_OK)
    Device.Diag.UsbUartErrorCnt++;
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

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
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  /*
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
    */
  return len;
}

/* UART-----------------------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *context)
{
  if(HAL_UART_Receive_IT(context, (uint8_t *)&UsbUartCharacter, 1) != HAL_OK)
    Device.Diag.UsbUartErrorCnt++;
  else
  {
    if(UsbUartRxBufferPtr < USB_UART_BUFFER_SIZE - 1)
    {
      if(UsbUartCharacter == UART_TERIMINATION_CHAR)
      {
        UsbUartRxBuffer[UsbUartRxBufferPtr] = '\0';
        strcpy(UsbUartTxBuffer, UsbUartParser(UsbUartRxBuffer));
        UsbUartRxBufferPtr = 0;
      }
      else
        UsbUartRxBuffer[UsbUartRxBufferPtr++] = UsbUartCharacter;
    }
    else
    {
      UsbUartRxBufferPtr = 0;
      memset(UsbUartRxBuffer,0x00, USB_UART_BUFFER_SIZE);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Device.Diag.UsbUartErrorCnt++;
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

  if(HAL_UART_Receive_IT(huart, (uint8_t *)&UsbUartCharacter, 1) != HAL_OK)
    Device.Diag.UsbUartErrorCnt++;
}

void UsbUartTxTask(void)
{
  uint8_t txLen = strlen(UsbUartTxBuffer);
  if(txLen != 0)
  {
    UsbUartTxBuffer[txLen] = UART_TERIMINATION_CHAR;
    UsbUartTxBuffer[txLen + 1] = '\0';

    HAL_UART_Transmit(&huart1, (uint8_t*) UsbUartTxBuffer, txLen + 1, 100);
    UsbUartTxBuffer[0] = 0;
  }
}

char* UsbUartParser(char *line)
{
  static char buffer[USB_UART_BUFFER_SIZE];
  char cmd[USB_UART_CMD_LENGTH];
  char arg1[USB_UART_ARG_LENGTH];
  char arg2[USB_UART_ARG_LENGTH];

  memset(buffer, 0x00, USB_UART_BUFFER_SIZE);
  memset(cmd,0x00, USB_UART_CMD_LENGTH);
  memset(arg1,0x00, USB_UART_ARG_LENGTH);
  memset(arg2,0x00, USB_UART_ARG_LENGTH);

  sscanf(line, "%s",cmd);

  if(!strcmp(cmd, "*IDN?"))
  {
    sprintf(buffer, "*IDN? %s", DEVICE_NAME);
  }
  else if(!strcmp(cmd, "*OPC?"))
  {
    strcpy(buffer, "*OPC? OK");
  }
  else if(!strcmp(cmd, "FW?"))
  {
    sprintf(buffer, "FW? %s", DEVICE_FW);
  }
  else if(!strcmp(cmd, "UID?"))
  {
    sprintf(buffer, "UID? %4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
  }
  else if(!strcmp(cmd, "PCB?"))
  {
    sprintf(buffer, "PCB? %s", DEVICE_PCB);
  }
  else if(!strcmp(cmd,"UPTIME?"))
  {
     sprintf(buffer, "UPTIME? %08lX", Device.Diag.UpTimeSec);
  }
  else if(!strcmp(cmd,"DI?"))
  {
     sprintf(buffer, "DI? %08hX", Device.DI);
  }
  else if(!strcmp(cmd,"DO?"))
  {
     sprintf(buffer, "DO? %08hX", Device.DO);
  }
  else if(!strcmp(cmd,"UE?"))
  {
    sprintf(buffer, "UE? %08lX", Device.Diag.UsbUartErrorCnt);
  }
  else if(!strcmp(cmd,"DO"))
  {
    sscanf(line, "%s %s",cmd, arg1);
    Device.DO = strtol(arg1, NULL, 16);
    strcpy(buffer, "DO OK");
  }
  else
  {
    Device.Diag.UsbUartUnknwonCnt++;
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
