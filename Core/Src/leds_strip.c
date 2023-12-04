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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/*
 *
 *
 * PA8
 *
 * DMA1 CH2
 */

void LedsInit(TIM_HandleTypeDef *htim)
{

  //LL_TIM_EnableARRPreload(htim);
  /*
   * 1/48MHz/60000 = 1.25ms
   */
  //LL_TIM_SetAutoReload(htim->Instance, 60000);



  HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);


  HAL_TIM_PWM_GetState(htim);
}


void LedsTask()
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //PA4
  HAL_GPIO_TogglePin(DBG_PERIOD_CLK_GPIO_Port, DBG_PERIOD_CLK_Pin);

}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{

}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
