/*
 * Platform.h
 *
 *  Created on: 4 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_PLATFORM_H_
#define INC_PLATFORM_H_

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32f7xx_hal.h"

#define DEFAULT_MOTOR_FREQUENCY_HZ 60
#define ENCODER_RESOLUTION 131072
#define POSITION_ERROR 92

typedef struct {
	uint32_t ENCODER_OFFSET;
	uint32_t current_pos;
	uint32_t test_spec_pos;
	uint32_t start_spec_pos;
	uint32_t end_spec_pos;
	uint32_t end_temp_pos;
	uint32_t accel_spec_pos;
	uint32_t inc_pos;
} encoder;

typedef struct  {
	bool platform_id;
	uint16_t motor_freq_test_Hz;
	uint16_t motor_freq_Hz;
	uint16_t motor_freq_def_Hz;
	uint16_t measurement_res;
	uint8_t measurement_interval;
	TIM_HandleTypeDef* motor_tim;
	encoder encoder;
} platform;

extern platform horizontal, vertical;

void setEncoderPollFrequency(uint16_t frequency_mcs);
//void changeMotorDirection()

#endif /* INC_PLATFORM_H_ */
