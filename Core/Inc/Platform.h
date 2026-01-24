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

#define DEFAULT_MOTOR_FREQUENCY_HZ 200
#define PICK_UP_MOTOR_FREQUENCY_HZ 10
#define ENCODER_RESOLUTION 131072
#define POSITION_ERROR 92
#define AHB1_TIMER_CLOCK_MHz 108

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
extern bool is_motor_moving, is_backlash_passed, is_req_freq_reach;;
extern uint16_t current_motor_freq, target_motor_freq, motor_freq_inc_hz;

void setEncoderPollFrequency(uint16_t frequency_mcs);
void startMotorRotation(uint8_t motor_id, uint32_t last_encoder_data);
void checkBacklash();
void changeMotorDirection__(platform *cur_platf, uint32_t target_position);
uint32_t calculateEncPosition__(platform* cur_platf, uint32_t encoder_position);
void setPlatformParam__(platform* cur_platf);
void setMotorFrequency__(platform* cur_platf, uint16_t motor_frequency);
void setMotorFrequency(bool chosen_drv, uint16_t motor_frequency);
void stopMotorRotation(uint8_t motor_id);
//void changeMotorDirection()

#endif /* INC_PLATFORM_H_ */
