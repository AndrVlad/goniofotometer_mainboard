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
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000) /* Базовый адрес сектора для хранения смещения нуля лимба */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08018000) /* Базовый адрес сектора для хранения смещения энкодеров */

void FlashInit();
void WriteToFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_program, uint32_t erase_sector_num);
void ReadFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_read);

extern uint32_t page_error;

#endif /* INC_FLASHUTILS_H_ */
