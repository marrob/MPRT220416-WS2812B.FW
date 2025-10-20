/*
 * ws2812b.h
 *
 *  Created on: Dec 3, 2023
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

  #define WS2812B_BLACK         0x000000
  #define WS2812B_WHITE         0xFFFFFF
  #define WS2812B_BLUE          0x0000FF
  #define WS2812B_GREEN         0x00FF00
  #define WS2812B_RED           0xFF0000

  #define WS2812B_SYNT_01       0x8A04ED
  #define WS2812B_SYNT_02       0xEF9AF2
  #define WS2812B_SYNT_03       0x240C76
  #define WS2812B_SYNT_04       0x570296
  #define WS2812B_SYNT_05       0x831187
  #define WS2812B_SYNT_06       0x0C0C0C

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void WS2812B_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma, uint32_t *colorBuffer, uint32_t led_count);
void WS2812B_Task(void);
void WS2812B_SetBrightness(uint8_t value);
uint8_t WS2812B_GetBrightness();

#endif /* INC_WS2812B_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
