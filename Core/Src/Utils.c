/*
 * Utils.c
 *
 *  Created on: 5 дек. 2025 г.
 *      Author: vlado
 */

#include "Utils.h"

void clearBuffer(uint8_t *buf, uint8_t size){
	for(uint8_t i = 0; i < size; i++) {
		buf[i] = 0;
	}
}

void clearSpecifiedElemOfBuffer(uint8_t *buf, uint8_t size, uint8_t start_clear_pos){
	for(uint8_t i = start_clear_pos; i < size; i++) {
		buf[i] = 0;
	}
}

uint32_t calculateMedianVal(uint32_t* buf, uint16_t size, uint8_t limit) {
	uint8_t end_limit = size - (limit + 1);
	uint64_t median_val = 0;
	uint32_t result;
	for (uint16_t i = limit; i <= end_limit; i++) {
		median_val += buf[i];
	}
	result = median_val / (size - limit*2);
	return result;
}

void swap(uint32_t* a, uint32_t* b) {
    uint32_t tmp = *a;
    *a = *b;
    *b = tmp;
}

void bubbleSort(uint32_t *buf, uint16_t size)
{
	while (size--)
	{
		bool swapped = false;

		for (int i = 0; i < size; i++)
		{
			if (buf[i] > buf[i + 1])
			{
				swap(&buf[i], &buf[i + 1]);
				swapped = true;
			}
		}

		if (swapped == false)
			break;
	}
}


