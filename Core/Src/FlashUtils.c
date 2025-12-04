/*
 * FlashUtils.c
 *
 *  Created on: 29 нояб. 2025 г.
 *      Author: Andrianov Vladislav Alekseevich
 */

#include "FlashUtils.h"
#include "stm32f7xx_hal.h"


uint32_t page_error = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;

void FlashInit() {
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_4;
	EraseInitStruct.NbSectors     = 1;
}

void WriteToFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_program) {

	HAL_FLASH_Unlock();

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK) {
	      //error handler of erasing flash
	      return;
	  }

	  uint8_t address_inc = 0;

	  if (type_of_program == FLASH_TYPEPROGRAM_WORD) {
		  address_inc = 4;
	  } else if (type_of_program == FLASH_TYPEPROGRAM_HALFWORD) {
		  address_inc = 2;
	  }

	  for(uint8_t i = 0; i < data_size; i++)
	  {
		  if(HAL_FLASH_Program(type_of_program, address, data[i]) != HAL_OK) {
	            // error handler of programming flash
	            return;
	      }
	           address += address_inc;
	   }

	   HAL_FLASH_Lock();
}

void ReadFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_read) {

	uint8_t address_inc = 0;
	uint32_t dig32 = 0;
	if (type_of_read == FLASH_TYPEPROGRAM_WORD) {
	    address_inc = 4;
	} else if (type_of_read == FLASH_TYPEPROGRAM_HALFWORD) {
		address_inc = 2;
	}

	for(uint16_t i = 0; i < data_size; i++) {

		dig32 = *(uint32_t*)address;
		data[i] = dig32;
	    address += address_inc;
	  }
}
