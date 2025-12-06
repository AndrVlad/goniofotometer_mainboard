/*
 * PD_Communication.c
 *
 *  Created on: 6 дек. 2025 г.
 *      Author: vlado
 */
#include "PD_Communication.h"
#include "stm32f7xx_hal.h"
#include "Common.h"
#include "HardwareUtils.h"


bool wait_adc_data_flag = 0;
uint8_t uart1_rx_buffer[5] = {0};
uint8_t data_buf_counter = 0;
uint8_t adc_data_buf[33] = {0};
uint8_t data_elem_cnt = 1;
bool wait_flag = 0;

void pollPhotodetector() {
	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	wait_adc_data_flag = 1;
	usDelay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}


