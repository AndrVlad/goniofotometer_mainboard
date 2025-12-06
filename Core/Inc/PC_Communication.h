/*
 * PC_Communication.h
 *
 *  Created on: 5 дек. 2025 г.
 *      Author: vlado
 */

#ifndef INC_PC_COMMUNICATION_H_
#define INC_PC_COMMUNICATION_H_

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"

enum data { NONE_, _READY_, SOME_PACKETS};
extern enum data data_status;
extern bool stop_poll;

#endif /* INC_PC_COMMUNICATION_H_ */
