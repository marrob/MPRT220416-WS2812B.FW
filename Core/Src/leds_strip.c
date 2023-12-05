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


#define LED_CFG_COUNT                           1
#define LED_CFG_BYTES_PER_LED                   3


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*
 * A DMA buffer mérete, ide kerül minden egyes bithez tartozó PWM érték ami 32 biten tárolódik.
 *
 * - pl: RGB LED, 3 bájton tárolja a szineit igy az 24 bit
 * - mindig 1 LED értékét tartjuak a DMA bufferében
 *
 *
 */
uint32_t dmaBuffer[8 * LED_CFG_BYTES_PER_LED];
__IO uint8_t ready;
TIM_HandleTypeDef *_htim;


uint32_t leds_color_data[] =
{
    //xxBGR
    0x0000000FF
};

/* Private function prototypes -----------------------------------------------*/
void DMATransferCplt(DMA_HandleTypeDef * _hdma);
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
  //LL_TIM_EnableARRPreload(htim);
  /*
   * 1/48MHz/60000 = 1.25ms
   */
  //LL_TIM_SetAutoReload(htim->Instance, 60000);



 // HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start_IT(htim,TIM_CHANNEL_1);


 // HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);

  HAL_TIM_PWM_GetState(htim);

  /*
  HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                          uint16_t Length);
  HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
  */

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


  //HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));

  HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID, DMATransferCplt );

  //__HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);;

  int dmaBufferLen = sizeof(dmaBuffer) / sizeof(dmaBuffer[0]);
  HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, dmaBuffer, dmaBufferLen);
}


void DMATransferCplt(DMA_HandleTypeDef * _hdma)
{
  HAL_GPIO_TogglePin(DBG_PERIOD_CLK_GPIO_Port, DBG_PERIOD_CLK_Pin);
  ready = 1;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

  HAL_GPIO_TogglePin(DBG_PERIOD_CLK_GPIO_Port, DBG_PERIOD_CLK_Pin);
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);

}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{

}


void LedsTask(void)
{
  if(ready)
  {
    //HAL_TIM_PWM_Start_DMA(_htim, TIM_CHANNEL_1, dmaBuffer, sizeof(dmaBuffer) / sizeof(dmaBuffer[0]) );
  }

}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
