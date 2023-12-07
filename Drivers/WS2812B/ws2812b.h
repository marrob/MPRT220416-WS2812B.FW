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

void LedsInit(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
void LedsTask(void);

#endif /* INC_WS2812B_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
