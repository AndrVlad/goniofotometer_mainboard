/*
 * PlatformMeasurements.h
 *
 *  Created on: 1 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_PLATFORMMEASUREMENTS_H_
#define INC_PLATFORMMEASUREMENTS_H_

#include "main.h"

typedef struct {
	uint32_t ENCODER_OFFSET;
	uint32_t current_pos;
	uint32_t test_spec_pos;
	uint32_t start_spec_pos;
	uint32_t end_spec_pos;
	uint32_t accel_spec_pos;
} encoder;

typedef struct  {
	uint16_t motor_freq_test_Hz;
	uint16_t motor_freq_Hz;
	encoder encoder;
} platform;

extern platform horizontal, vertical;


#endif /* INC_PLATFORMMEASUREMENTS_H_ */
