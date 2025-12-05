/*
 * FlashUtils.h
 *
 *  Created on: 28 нояб. 2025 г.
 *      Author: vlado
 */

#ifndef INC_FLASHUTILS_H_
#define INC_FLASHUTILS_H_

//#include "main.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include <stdint.h>

#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08018000) /* Base address of Sector 3, 32 Kbytes */

void FlashInit();
void WriteToFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_program);
void ReadFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_read);

extern uint32_t page_error;

#endif /* INC_FLASHUTILS_H_ */
