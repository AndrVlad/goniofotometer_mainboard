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
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

enum action { NONE, HORIZONTAL, VERTICAL, HEMISPHERE, LIGHT_POWER, CALIBRATION,
				TEST_TURN, TEST_ROTATION, TEST_ANGLE_OFFSET, MOVING };

#endif /* INC_COMMON_H_ */
