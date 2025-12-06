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

#define ACCEL_OFFSET 5 // values in ark degrees
#define ENCODER_TOLERANCE 46

extern uint8_t start_end_angle_item[8];
extern uint16_t measurement_res_item[8];

void InitPlatformMeasurement(platform* chosen_platf, uint8_t measurement_type, uint8_t start_interval, uint8_t end_interval, uint8_t resolution_pos);
void handleHorizVerticMeasurement();
#endif /* INC_PLATFORMMEASUREMENTS_H_ */
