/*
 * HardwareUtils.h
 *
 *  Created on: 4 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_HARDWAREUTILS_H_
#define INC_HARDWAREUTILS_H_

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

void usDelay(uint16_t useconds);
void setNVICPriority(uint8_t cur_action);
void resetNVICPriority();

#endif /* INC_HARDWAREUTILS_H_ */
