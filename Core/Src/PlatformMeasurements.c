/*
 * PlatformMeasurements.c
 *
 *  Created on: 5 дек. 2025 г.
 *      Author: vlado
 */

#include "PlatformMeasurements.h"
#include "Common.h"
#include "HardwareUtils.h"

platform* cur_platf;

typedef struct {
	bool reach_accel_pos;
	bool reach_start_pos;
	bool reach_end_pos;
} platf_meas_state;

platf_meas_state platf_state;

uint8_t start_end_angle_item[8] = {180,150,120,90,60,30,10,5};
uint16_t measurement_res_item[8] = {365,182,60,29,6,3,1};
static bool full_rotation = 0;

void InitPlatformMeasurement(platform* chosen_platf, uint8_t measurement_type, uint8_t start_interval, uint8_t end_interval, uint8_t resolution_pos) {

	uint16_t start_angle, end_angle, accel_angle;
	uint32_t start_position, end_position, end_position_tmp, accel_position;

	// definition of chosen platform
	cur_platf = chosen_platf;

	platf_state.reach_start_pos = 0;
	platf_state.reach_end_pos = 0;
	platf_state.reach_accel_pos = 0;

	// set start angle of measurement
	start_angle = abs(start_end_angle_item[start_interval-1] - 180);

	//start_angle = start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
	// calculate offset of the measurement from specified start position
	/*
	if ((start_angle + start_angle_offset_1) > 360) {
		start_angle = (start_angle + start_angle_offset_1) - 360;
	} else {
		start_angle = start_angle + start_angle_offset_1;
	} */

	// set end angle of measurement
	end_angle = start_end_angle_item[end_interval-1] + 180;
	/*
	if ((end_angle + start_angle_offset_1) > 360) {
		end_angle = (end_angle + start_angle_offset_1) - 360;
	} else {
		end_angle = end_angle + start_angle_offset_1;
	}
	*/

	if (start_angle == 0 && end_angle == 360) {
		full_rotation = 1;
	}

	// calculate acceleration offset position

	if ((start_angle - ACCEL_OFFSET) < 0) {
		accel_angle = abs(start_angle - ACCEL_OFFSET);
		accel_angle = 360 - accel_angle;
	} else {
		accel_angle = start_angle - ACCEL_OFFSET;
	}

	// set the measurement resolution
	cur_platf->measurement_res = measurement_res_item[resolution_pos - 1];

	setPlatformParam__(cur_platf);

	// set acceleration position
	accel_position = (accel_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
	cur_platf->encoder.accel_spec_pos = calculateEncPosition__(cur_platf,accel_position);
	changeMotorDirection__(cur_platf, cur_platf->encoder.accel_spec_pos);

	// set start position of measurement
	start_position = (start_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
	cur_platf->encoder.start_spec_pos = calculateEncPosition__(cur_platf,start_position);

	// set encoder position increment
	if ((start_position + cur_platf->measurement_res) > ENCODER_RESOLUTION) {
		cur_platf->encoder.inc_pos = (start_position + cur_platf->measurement_res) - ENCODER_RESOLUTION;
	} else {
		cur_platf->encoder.inc_pos = start_position + cur_platf->measurement_res;
	}

	// set end position of measurement
	if (full_rotation) {
		end_position_tmp = (end_angle * ENCODER_RESOLUTION) / 360;
		cur_platf->encoder.end_temp_pos = calculateEncPosition__(cur_platf,end_position_tmp);
		end_position = ((end_angle-4) * ENCODER_RESOLUTION) / 360;
		cur_platf->encoder.end_spec_pos = calculateEncPosition__(cur_platf,end_position);
	} else {
		end_position = (end_angle * ENCODER_RESOLUTION) / 360;
		cur_platf->encoder.end_spec_pos = calculateEncPosition__(cur_platf,end_position);
	}

	setNVICPriority(cur_action);

	trans_states = 1;

	cur_action = measurement_type;

	ready_status = BUSY_;

	// start measurement
	HAL_TIM_Base_Start_IT(&htim7);					// start poll encoder
	HAL_TIM_Base_Start_IT(cur_platf->motor_tim); 	// start motor moving

}
