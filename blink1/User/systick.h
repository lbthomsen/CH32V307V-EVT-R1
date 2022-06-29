/*
 * systick.h
 *
 *  Created on: Jun 29, 2022
 *      Author: lth
 */

#ifndef USER_SYSTICK_H_
#define USER_SYSTICK_H_

#include "ch32v30x.h"

void Systick_Init(void);

uint32_t GetTick();

#endif /* USER_SYSTICK_H_ */
