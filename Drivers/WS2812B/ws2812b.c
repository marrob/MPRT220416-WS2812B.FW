/*
 * ws2812b.c
 *
 *  Created on: Dec 3, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_tim.h"

#include <stdio.h>
#include <ws2812b.h>
/* Private define ------------------------------------------------------------*/


#define WS2812B_LED_COUNT          3
#define WS2812B_LED_COLORS         3
#define WS2812B_LED_BITS_OF_COLOR  8


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*
 * A DMA buffer mérete, ide kerül minden egyes bithez tartozó PWM érték ami 32 biten tárolódik.
 *
 * - pl: RGB LED, 3 bájton tárolja a szineit igy az 24 bit
 * - mindig 1 LED értékét tartjuak a DMA bufferében
 *
 */

uint32_t dmaBuffer[1 + WS2812B_LED_BITS_OF_COLOR * WS2812B_LED_COLORS + 1];
__IO uint8_t _updateReady;
__IO uint8_t _ledIndex;
TIM_HandleTypeDef *_htim;
DMA_HandleTypeDef *_hdma;


uint32_t colorBuffer[WS2812B_LED_COUNT] =
{
    //0xXXRRGGBB
    0x0000FF,
    0x00FF00,
    0xFF0000,
};



/* Private function prototypes -----------------------------------------------*/
void LedUpdateStart(uint32_t ledIdx);
/* Private user code ---------------------------------------------------------*/

void LedsInit(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma)
{
  _htim = htim;
  _hdma = hdma;
  _updateReady = 1;

/*
  for(int i = 0; i < WS2812B_LED_COUNT; i++)
    colorBuffer[i] = 0xFFFFFFF;
    */
}

/*** For WS2812B ***/
void LedUpdateStart(uint32_t ledIdx)
{
  const uint32_t arr = _htim->Instance->ARR + 1;
  const uint32_t pulse_high = (3 * arr / 4) - 1;
  const uint32_t pulse_low = (1 * arr / 4) - 1;
  uint32_t r, g, b;


  /*** 0xXXRRGGBB -> ***/
  b = (uint8_t)(colorBuffer[ledIdx]); // blue
  g = (uint8_t)(colorBuffer[ledIdx] >> 8); //green
  r = (uint8_t)(colorBuffer[ledIdx] >> 16); // red

  /*** 1LED -> DMA Buffer Conversion ***/
  for (size_t bit = 0; bit < 8; bit++)
  {
    /***  0xXXRRGGBB -> []{ 0x00, 0xG7...0x0G0, 0xR7... 0xR0, 0xB7..0xB0, 0x00 } ***/
    dmaBuffer[bit + 1] =        (g & (1 << (7 - bit))) ? pulse_high : pulse_low;
    dmaBuffer[8 + bit + 1] =    (r & (1 << (7 - bit))) ? pulse_high : pulse_low;
    dmaBuffer[16 + bit + 1] =   (b & (1 << (7 - bit))) ? pulse_high : pulse_low;
  }
  int dmaBufferLen = sizeof(dmaBuffer) / sizeof(dmaBuffer[0]);
HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_1, dmaBuffer, dmaBufferLen);
/*** Disable not used DMA fucntions ***/
__HAL_DMA_DISABLE_IT (_hdma, DMA_IT_HT);
__HAL_DMA_DISABLE_IT(_hdma, DMA_IT_TE);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);

  if(_ledIndex < WS2812B_LED_COUNT - 1 )
  {
    _ledIndex++;
    LedUpdateStart(_ledIndex);

  }
  else
  {
    _ledIndex = 0;
    _updateReady = 1;
    HAL_GPIO_TogglePin(DIAG_LED_STRING_UPDT_CLK_GPIO_Port, DIAG_LED_STRING_UPDT_CLK_Pin); //PA5
  }

  HAL_GPIO_TogglePin(DIAG_LED_UPDT_CLK_GPIO_Port, DIAG_LED_UPDT_CLK_Pin);//PA4
}


void LedsTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 250)
  {
    HAL_GPIO_WritePin(DIAG_LED_UPDT_CLK_GPIO_Port, DIAG_LED_UPDT_CLK_Pin, GPIO_PIN_RESET);//PA4
    HAL_GPIO_WritePin(DIAG_LED_STRING_UPDT_CLK_GPIO_Port, DIAG_LED_STRING_UPDT_CLK_Pin, GPIO_PIN_RESET); //PA5

    timestamp = HAL_GetTick();

    if(_updateReady)
    {
      _updateReady = 0;
      LedUpdateStart(_ledIndex);
    }
  }
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
