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

bool is_motor_moving = false;
bool is_backlash_passed = false;
uint32_t last_encoder_pos = 0;
uint16_t current_motor_freq, target_motor_freq;

void startMotorRotation(uint8_t motor_id, uint32_t last_encoder_data) {

	setMotorFrequency(motor_id,PICK_UP_MOTOR_FREQUENCY_HZ);
	last_encoder_pos = last_encoder_data;

	HAL_TIM_Base_Start_IT(&htim7); // start encoder polling

	if (motor_id) {
		HAL_TIM_Base_Start_IT(&htim3);
	} else {
		HAL_TIM_Base_Start_IT(&htim2);
	}

	is_motor_moving = true;
	is_backlash_passed = false;
}

void checkBacklash(uint32_t encoder_data) {

	if ((encoder_data >= (last_encoder_pos + 25)) || (encoder_data <= (last_encoder_pos - 25))) {
		is_backlash_passed = true;
		// start accel_timer
		HAL_TIM_Base_Start_IT(&htim9);
	}
}

void stopMotorRotation(uint8_t motor_id) {

	HAL_TIM_Base_Stop_IT(&htim2); // stop motor
	HAL_TIM_Base_Stop_IT(&htim7); // stop encoder polling
	is_motor_moving = false;
	target_motor_freq = DEFAULT_MOTOR_FREQUENCY_HZ;
}

void setMotorFrequency(bool chosen_drv, uint16_t motor_frequency) {

	uint32_t tim_clock = 0;
	tim_clock = (AHB1_TIMER_CLOCK_MHz * 1000000);

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	if (chosen_drv) { // chosen second motor
		htim3.Instance->ARR = ((tim_clock/(htim3.Instance->PSC + 1))/motor_frequency)-1;
	} else {		// chosen first motor
		htim2.Instance->ARR = ((tim_clock/(htim2.Instance->PSC + 1))/motor_frequency) - 1;
	}

	current_motor_freq = motor_frequency;
}

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

void setMotorFrequency__(platform* cur_platf, uint16_t motor_frequency) {

	uint32_t tim_clock = 0;
	tim_clock = (AHB1_TIMER_CLOCK_MHz * 1000000);

	if (cur_platf->platform_id == VERT_PL) {
		htim3.Instance->ARR = ((tim_clock/(htim3.Instance->PSC + 1))/motor_frequency) - 1;
	} else {
		htim2.Instance->ARR = ((tim_clock/(htim2.Instance->PSC + 1))/motor_frequency) - 1;
	}
}

