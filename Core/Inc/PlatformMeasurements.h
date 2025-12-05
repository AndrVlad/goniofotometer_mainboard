/*
 * PlatformMeasurements.h
 *
 *  Created on: 5 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_PLATFORMMEASUREMENTS_H_
#define INC_PLATFORMMEASUREMENTS_H_


#include "Platform.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

extern uint8_t start_end_angle_item[8];
extern uint16_t measurement_res_item[8];

typedef struct {
	bool reach_accel_pos;
	bool reach_start_pos;
	bool reach_end_pos;
} platf_meas_state;

platf_meas_state platf_state;

#endif /* INC_PLATFORMMEASUREMENTS_H_ */
