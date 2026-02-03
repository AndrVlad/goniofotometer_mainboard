/*
 * Common.h
 *
 *  Created on: 4 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

enum action { NONE, HORIZONTAL, VERTICAL, HEMISPHERE, LIGHT_POWER, CALIBRATION,
				TEST_TURN, TEST_ROTATION, TEST_ANGLE_OFFSET, MOVING };
enum platform_type {HORIZ_PL,VERT_PL};
enum status { ERROR_, READY_, BUSY_ };

extern bool trans_states;
extern enum action cur_action;
extern enum status ready_status;

#endif /* INC_COMMON_H_ */
