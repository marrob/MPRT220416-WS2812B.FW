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

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void WS2812B_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma, uint32_t *colorBuffer, uint32_t led_count);
void WS2812B_Task(void);
void WS2812B_SetBrightness(uint8_t value);
uint8_t WS2812B_GetBrightness();

#endif /* INC_WS2812B_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
