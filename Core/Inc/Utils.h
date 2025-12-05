/*
 * Utils.h
 *
 *  Created on: 5 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

void clearBuffer(uint8_t *buf, uint8_t size);
void clearSpecifiedElemOfBuffer(uint8_t *buf, uint8_t size, uint8_t start_clear_pos);
void bubbleSort(uint32_t *buf, uint16_t size);
uint32_t calculateMedianVal(uint32_t* buf, uint16_t size, uint8_t limit);

#endif /* INC_UTILS_H_ */
