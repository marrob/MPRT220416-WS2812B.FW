/*
 * leds_strip.c
 *
 *  Created on: Dec 3, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "leds_strip.h"
#include "stm32f1xx_ll_tim.h"

#include <stdio.h>
/* Private define ------------------------------------------------------------*/


#define LED_COUNT                               6
#define LED_CFG_BYTES_PER_LED                   3


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*
 * A DMA buffer mérete, ide kerül minden egyes bithez tartozó PWM érték ami 32 biten tárolódik.
 *
 * - pl: RGB LED, 3 bájton tárolja a szineit igy az 24 bit
 * - mindig 1 LED értékét tartjuak a DMA bufferében
 */

uint32_t dmaBuffer[8 * LED_CFG_BYTES_PER_LED];
__IO uint8_t _updateReady;
__IO uint8_t _ledIndex;

TIM_HandleTypeDef *_htim;
DMA_HandleTypeDef *_hdma;


uint32_t leds_color_data[] =
{
    //xxRGB
    0x0000000FF,
    0x00000FF00,
    0x000FF0000,

    0x000FF0000,
    0x00000FF00,
    0x0000000FF
};

/* Private function prototypes -----------------------------------------------*/
void DMATransferCplt(DMA_HandleTypeDef * _hdma);
void DMATransferStart(uint32_t ledx);
/* Private user code ---------------------------------------------------------*/

/*
 *
 *
 * PA8
 *
 * DMA1 CH2
 */
void LedsInit(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma)
{

  _htim = htim;
  _hdma = hdma;

  //LL_TIM_EnableARRPreload(htim);
  /*
   * 1/48MHz/60000 = 1.25ms
   */
  //LL_TIM_SetAutoReload(htim->Instance, 60000);



 // HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start_IT(htim,TIM_CHANNEL_1);


 // HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);

//  HAL_TIM_PWM_GetState(htim);

  /*
  HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                          uint16_t Length);
  HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
  */
/*
  const uint32_t arr = htim->Instance->ARR + 1;
  const uint32_t pulse_high = (3 * arr / 4) - 1;
  const uint32_t pulse_low = (1 * arr / 4) - 1;
  uint32_t r, g, b;

  for(int ledx = 0 ; ledx < sizeof(leds_color_data) / sizeof(leds_color_data[0]); ledx++)
  {

    r = *((uint8_t *)leds_color_data + ledx * LED_CFG_BYTES_PER_LED + 0);
    g = *((uint8_t *)leds_color_data + ledx * LED_CFG_BYTES_PER_LED + 1);
    b = *((uint8_t *)leds_color_data + ledx * LED_CFG_BYTES_PER_LED + 2);

    for (size_t bit = 0; bit < 8; bit++)
    {
        dmaBuffer[bit] =        (g & (1 << (7 - bit))) ? pulse_high : pulse_low;
        dmaBuffer[8 + bit] =    (r & (1 << (7 - bit))) ? pulse_high : pulse_low;
        dmaBuffer[16 + bit] =   (b & (1 << (7 - bit))) ? pulse_high : pulse_low;
    }
  }
*/

  //HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));

 // HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID, DMATransferCplt );

 // __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);


 // DMATransferStart(_ledIndex);

  _updateReady = 1;

}

void DMATransferStart(uint32_t ledx)
{
  const uint32_t arr = _htim->Instance->ARR + 1;
  const uint32_t pulse_high = (3 * arr / 4) - 1;
  const uint32_t pulse_low = (1 * arr / 4) - 1;
  uint32_t r, g, b;

    r = (uint8_t)(leds_color_data[ledx]);
    g = (uint8_t)(leds_color_data[ledx] >> 8);
    b = (uint8_t)(leds_color_data[ledx] >> 16);

    //1bit -> 32bit conversion
    for (size_t bit = 0; bit < 8; bit++)
    {
      dmaBuffer[bit] =        (r & (1 << (7 - bit))) ? pulse_high : pulse_low;
      dmaBuffer[8 + bit] =    (g & (1 << (7 - bit))) ? pulse_high : pulse_low;
      dmaBuffer[16 + bit] =   (b & (1 << (7 - bit))) ? pulse_high : pulse_low;
    }
    int dmaBufferLen = sizeof(dmaBuffer) / sizeof(dmaBuffer[0]);
    HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_1, dmaBuffer, dmaBufferLen);
}


void DMATransferCplt(DMA_HandleTypeDef * _hdma)
{

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);

  if(_ledIndex < LED_COUNT - 1 )
  {
    _ledIndex++;
    __HAL_DMA_DISABLE_IT (_hdma, DMA_IT_HT);
    DMATransferStart(_ledIndex);
  }
  else
  {
    _ledIndex = 0;
    _updateReady = 1;
  }


   HAL_GPIO_TogglePin(DBG_PERIOD_CLK_GPIO_Port, DBG_PERIOD_CLK_Pin);

  //  __HAL_TIM_DISABLE(htim);
}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    //LL_DMA_EnableIT_TC

  //HAL_TIM_ACTIVE_CHANNEL_1

  //printf("itt jartam\r\n");
}


void LedsTask(void)
{
  static uint8_t color;

  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 250)
  {
    timestamp = HAL_GetTick();

    if(_updateReady)
    {
      _updateReady = 0;

      __HAL_DMA_DISABLE_IT (_hdma, DMA_IT_HT);

      leds_color_data[2] = color++;

      DMATransferStart(_ledIndex);
    }
  }
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
