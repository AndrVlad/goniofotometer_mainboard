/*
 * Platform.c
 *
 *  Created on: 4 дек. 2025 г.
 *      Author: vlado
 */
#include "Platform.h"
#include "Common.h"

platform horizontal = {.platform_id = HORIZ_PL};
platform vertical = {.platform_id = VERT_PL};

void setEncoderPollFrequency(uint16_t frequency_mcs) {
	htim7.Instance->ARR = frequency_mcs-1;
}

void changeMotorDirection__(platform *cur_platf, uint32_t target_position) {

	if (target_position < cur_platf->encoder.current_pos) {
		if ((cur_platf->encoder.current_pos - target_position) > (ENCODER_RESOLUTION - cur_platf->encoder.current_pos + target_position)) {
			if (cur_platf->platform_id == HORIZ_PL) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // moving forward
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // moving forward
			}
		} else {
			if (cur_platf->platform_id == HORIZ_PL) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			}
		}
	} else {
		if ((ENCODER_RESOLUTION - target_position + cur_platf->encoder.current_pos) < (target_position - cur_platf->encoder.current_pos)) {
			if (cur_platf->platform_id == HORIZ_PL) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			}
		} else {
			if (cur_platf->platform_id == HORIZ_PL) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // moving forward
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // moving forward
			}
		}
	}
}

uint32_t calculateEncPosition__(platform* cur_platf, uint32_t encoder_position) {

	uint32_t encoder_pos_ret = 0;

	if((encoder_position + cur_platf->encoder.ENCODER_OFFSET) > ENCODER_RESOLUTION) {
		encoder_pos_ret = (encoder_position + cur_platf->encoder.ENCODER_OFFSET) - ENCODER_RESOLUTION;
	} else {
		encoder_pos_ret = encoder_position + cur_platf->encoder.ENCODER_OFFSET;
	}

	if (encoder_pos_ret == 0) {
		encoder_pos_ret = POSITION_ERROR;
	}

	if (encoder_pos_ret == ENCODER_RESOLUTION) {
		encoder_pos_ret -= POSITION_ERROR;
	}

	return encoder_pos_ret;
}

void setPlatformParam__(platform* cur_platf) {
	switch(cur_platf->measurement_res) {
	case 365:
		cur_platf->motor_freq_Hz = 40;
		setEncoderPollFrequency(200);
		break;
	case 182:
		cur_platf->motor_freq_Hz = 40;
		setEncoderPollFrequency(200);
		break;
	case 60:
	case 29:
	case 6:
	case 3:
	case 1:
		cur_platf->motor_freq_Hz = 40;
		setEncoderPollFrequency(1000);
		break;
	}
}

