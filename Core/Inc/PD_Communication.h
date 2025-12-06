/*
 * PD_Communication.h
 *
 *  Created on: 6 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_PD_COMMUNICATION_H_
#define INC_PD_COMMUNICATION_H_

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

extern bool wait_adc_data_flag;
extern uint8_t uart1_rx_buffer[5];
extern uint8_t data_buf_counter;
extern uint8_t adc_data_buf[33];
extern uint8_t data_elem_cnt;
extern bool wait_flag;

void pollPhotodetector();

#endif /* INC_PD_COMMUNICATION_H_ */
