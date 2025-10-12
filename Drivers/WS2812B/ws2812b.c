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
#define WS2812B_LED_COLORS         3 //RGB
#define WS2812B_LED_BITS_OF_COLOR  8 //every color has 8 bit

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//.RAM-al mükodik a DMA STM32G051, és aligned 4-el nem tudom miért oldotta meg a problémat
//ellenkezo esetben nem indult el a Timer.
//STM32F103-eseten nem kellett a .RAM és aligned(4)
__attribute__((section(".RAM"), aligned(4))) uint32_t _dmaBuffer[1 + WS2812B_LED_BITS_OF_COLOR * WS2812B_LED_COLORS + 1];
uint32_t *_colorBuffer;

static __IO uint8_t _updateReady;
static __IO uint32_t _ledIndex;
static __IO uint32_t _ledCount;
static double _brightness;
static TIM_HandleTypeDef *_htim;
static DMA_HandleTypeDef *_hdma;

/*
uint32_t _colorBuffer[WS2812B_LED_COUNT] =
{
    //0xXXRRGGBB
    0x0000FF,
    0x00FF00,
    0xFF0000,
};
*/

/* Private function prototypes -----------------------------------------------*/
void WS2812B_UpdateStart(uint32_t ledIdx);
/* Private user code ---------------------------------------------------------*/

void WS2812B_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma, uint32_t *colorBuffer, uint32_t ledCount)
{
  _htim = htim;
  _hdma = hdma;
  _updateReady = 1;
  _brightness = 1;
  _ledCount = ledCount;
  _colorBuffer = colorBuffer;
}

//--- For WS2812B ---
//- A Timer-t a HAL_TIM_PWM_Start_DMA
//- Mituán a _dmaBuffer-ben lévő PWM kódokat kiküldi, meghivja a HAL_TIM_PWM_PulseFinishedCallback, hogy jöhet a kovetkező LED
void WS2812B_UpdateStart(uint32_t ledIdx)
{
  const uint32_t arr = _htim->Instance->ARR + 1;
  const uint32_t pulse_high = (3 * arr / 4) - 1;
  const uint32_t pulse_low = (1 * arr / 4) - 1;
  uint32_t r, g, b;
  uint32_t br_r, br_g, br_b;

  //--- 0xXXRRGGBB -> ---
  b = (uint8_t)(_colorBuffer[ledIdx]);
  g = (uint8_t)(_colorBuffer[ledIdx] >> 8);
  r = (uint8_t)(_colorBuffer[ledIdx] >> 16);

  br_b = (uint8_t) b * _brightness;
  br_g = (uint8_t) g * _brightness;
  br_r = (uint8_t) r * _brightness;

  //--- 1LED -> DMA Buffer Conversion ---/
  for (size_t bit = 0; bit < 8; bit++)
  {
    //--- 0xXXRRGGBB -> []{ 0x00, 0xG7...0x0G0, 0xR7... 0xR0, 0xB7..0xB0, 0x00 } ---
    _dmaBuffer[bit + 1] =        (br_g & (1 << (7 - bit))) ? pulse_high : pulse_low;
    _dmaBuffer[8 + bit + 1] =    (br_r & (1 << (7 - bit))) ? pulse_high : pulse_low;
    _dmaBuffer[16 + bit + 1] =   (br_b & (1 << (7 - bit))) ? pulse_high : pulse_low;
  }
  //int dmaBufferLen = sizeof(_dmaBuffer) / sizeof(_dmaBuffer[0]); //26
  HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_1, _dmaBuffer, sizeof(_dmaBuffer) / sizeof(_dmaBuffer[0]));
  //---Disable not used DMA fucntions
  __HAL_DMA_DISABLE_IT (_hdma, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(_hdma, DMA_IT_TE);
}

/*
 * Ez PWM buffer kikuldese után hivodik meg, ez WS2812B_LED_COLORS * 8+2  byte-ot jelent
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);

  if(_ledIndex < _ledCount - 1 )
  {
    _ledIndex++;
    WS2812B_UpdateStart(_ledIndex);
  }
  else
  {
    _updateReady = 1;
    HAL_GPIO_TogglePin(DIAG_LED_STRING_UPDT_CLK_GPIO_Port, DIAG_LED_STRING_UPDT_CLK_Pin); //PA5
  }
  HAL_GPIO_TogglePin(DIAG_LED_UPDT_CLK_GPIO_Port, DIAG_LED_UPDT_CLK_Pin);//PA4
}

//Minden hivás után ha már kész van egy LED firssitése, akkor inditja a kovetkezot
void WS2812B_Task(void)
{
  static uint32_t timestamp;

  HAL_GPIO_WritePin(DIAG_LED_UPDT_CLK_GPIO_Port, DIAG_LED_UPDT_CLK_Pin, GPIO_PIN_RESET);//PA4
  HAL_GPIO_WritePin(DIAG_LED_STRING_UPDT_CLK_GPIO_Port, DIAG_LED_STRING_UPDT_CLK_Pin, GPIO_PIN_RESET); //PA5

  if(HAL_GetTick() - timestamp > 20) //50Hz
  {
    timestamp = HAL_GetTick();
    if(_updateReady)
    {
      _ledIndex = 0;
      _updateReady = 0;
      WS2812B_UpdateStart(_ledIndex);
    }
  }
}

/*
 * 0..100%
 */
void WS2812B_SetBrightness(uint8_t value)
{
  _brightness = value / 100.0;
}

uint8_t WS2812B_GetBrightness()
{
  return _brightness * 100;
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
