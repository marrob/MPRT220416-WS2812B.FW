/*
 * ws2812b.c
 *
 *  Created on: Dec 3, 2023
 *      Author: marrob
 *
 *
 *Gyakori problémák:
 * - Az UEV periodusideje 2.5ms, kell hogy legyen így a PWM periodus ideje 1.25us (ehhez van egy help a light.xlsx-ben)
 * - A DMA Word széles (32bit) és Memory to Pheri!
 * - Nem -Og vel rendben müködik (Optimaziton for Debug)
 * - Volt egy olyan verzió ahol a fényerő WS2812B_UpdateStart-ban volt benne,
 *   de STM32G051-nél a szorzás miatt miatt sokat hibáztott ott. ezért kivettem belőle a fényerő állítás lehetőségét.
 * - A LED-ek sorrendje nem passzol a dokumentációban lévővel
 * - LED támogatás:
 *  - 4 vezetékes WS2815
 *  - 3 vezetékes WS2812B
 *  uC támogatás:
 *   -STM32F103,
 *   - STM32G051
 *
 * Az Output Compare Prelaodnak be kell, hogy legyen kapcsolva
 *
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <ws2812b.h>
/* Private define ------------------------------------------------------------*/
#define WS2812B_LED_COLORS         3 //RGB
#define WS2812B_LED_BITS_OF_COLOR  8 //every color has 8 bit

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t _dmaBuffer[1 + WS2812B_LED_BITS_OF_COLOR * WS2812B_LED_COLORS + 1] = {0};
uint32_t *_colorBuffer;

static __IO uint8_t _updateReady;
static __IO uint32_t _ledIndex;
static __IO uint32_t _ledCount;
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

  //--- 0xXXRRGGBB -> ---
  b = (uint8_t)(_colorBuffer[ledIdx]);
  g = (uint8_t)(_colorBuffer[ledIdx] >> 8);
  r = (uint8_t)(_colorBuffer[ledIdx] >> 16);

  //Ez a rész itt időkiritkus, ne tegyél ide pl szorzást stb...

  //--- 1LED -> DMA Buffer Conversion ---/
  for (size_t bit = 0; bit < 8; bit++)
  {
    //--- 0xXXRRGGBB -> []{ 0x00, 0xG7...0x0G0, 0xR7... 0xR0, 0xB7..0xB0, 0x00 } ---
    _dmaBuffer[bit + 1] =        (g & (1 << (7 - bit))) ? pulse_high : pulse_low;
    _dmaBuffer[8 + bit + 1] =    (r & (1 << (7 - bit))) ? pulse_high : pulse_low;
    _dmaBuffer[16 + bit + 1] =   (b & (1 << (7 - bit))) ? pulse_high : pulse_low;
  }
  // int dmaBufferLen = sizeof(_dmaBuffer) / sizeof(_dmaBuffer[0]); //26
  HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_1, _dmaBuffer, sizeof(_dmaBuffer) / sizeof(_dmaBuffer[0]));
  //--- Disable not used DMA fucntions ---
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
//    HAL_GPIO_TogglePin(DIAG_LED_STRING_UPDT_CLK_GPIO_Port, DIAG_LED_STRING_UPDT_CLK_Pin); //PA5
  }
//  HAL_GPIO_TogglePin(DIAG_LED_UPDT_CLK_GPIO_Port, DIAG_LED_UPDT_CLK_Pin);//PA4
}

//Minden hivás után ha már kész van egy LED firssitése, akkor inditja a kovetkezot
void WS2812B_Task(void)
{
  static uint32_t timestamp;

//  HAL_GPIO_WritePin(DIAG_LED_UPDT_CLK_GPIO_Port, DIAG_LED_UPDT_CLK_Pin, GPIO_PIN_RESET);//PA4
//  HAL_GPIO_WritePin(DIAG_LED_STRING_UPDT_CLK_GPIO_Port, DIAG_LED_STRING_UPDT_CLK_Pin, GPIO_PIN_RESET); //PA5

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
void WS2812B_SetBrightness(uint8_t brightness_percent, uint32_t color_pattern_buffer[], size_t size)
{
  if (brightness_percent > 100)
      brightness_percent = 100;

  float scale = brightness_percent / 100.0f;

  for (size_t i = 0; i < size; i++)
  {
    uint32_t color = color_pattern_buffer[i];

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = (uint8_t)(r * scale);
    g = (uint8_t)(g * scale);
    b = (uint8_t)(b * scale);

    color_pattern_buffer[i] = (r << 16) | (g << 8) | b;
  }
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
