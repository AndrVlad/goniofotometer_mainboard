/*
 * HardwareUtils.c
 *
 *  Created on: 4 дек. 2025 г.
 *      Author: vlado
 */

#include "HardwareUtils.h"
#include "stm32f7xx_hal.h"
#include "Common.h"

void usDelay(uint16_t useconds)
{
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  while(__HAL_TIM_GET_COUNTER(&htim6) < useconds);
}

void setNVICPriority(uint8_t cur_action) {
	switch(cur_action) {
	case CALIBRATION:
	case LIGHT_POWER:
	  /* DMA1_Stream0_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	  /* DMA1_Stream1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 1);
	  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	  /* DMA2_Stream0_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	  /* DMA2_Stream1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 1);
	  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

	  HAL_NVIC_SetPriority( TIM1_UP_TIM10_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ( TIM1_UP_TIM10_IRQn);

		break;
	case HORIZONTAL:
	case VERTICAL:
		/* DMA2_Stream2_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART1_IRQn);

		HAL_NVIC_SetPriority(TIM7_IRQn, 2, 1);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);

		/* DMA1_Stream0_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
		  /* DMA1_Stream1_IRQn interrupt configuration */
		  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 1);
		  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
		  /* DMA2_Stream0_IRQn interrupt configuration */
		  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
		  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
		  /* DMA2_Stream1_IRQn interrupt configuration */
		  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 1);
		  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

		break;
	default:
		break;
	}
}

void resetNVICPriority() {
	  /* DMA1_Stream0_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	  /* DMA1_Stream1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	  /* DMA1_Stream5_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	  /* DMA2_Stream0_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 1);
	  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	  /* DMA2_Stream1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
	  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	  /* DMA2_Stream2_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);

		HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
}
