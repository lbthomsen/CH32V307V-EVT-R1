/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : Lars Boegild Thomsen <lbthomsen@gmail.com>
* Version            : V1.0.0
* Description        : Main program body.
* Copyright (c) 2022 Lars Boegild Thomsen
* SPDX-License-Identifier: MIT
*******************************************************************************/

#include <stdio.h>
#include <math.h>
#include "ch32v30x.h"
#include "systick.h"
#include "debug.h"

/* Global typedef */

/* Global define */
#define BUFFER_SIZE 48 // 48 kHz every 1 ms

/* Global Variable */
uint16_t buf1[2 * BUFFER_SIZE], buf2[2 * BUFFER_SIZE];


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	Systick_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n", SystemCoreClock);

	printf("This is oscillator example\r\n");

	uint32_t now = 0, last_tick = 0, last_calc = 0;

	while(1)
    {
		now = GetTick();

		if (now - last_tick >= 1000) {
			printf("Tick %lu\n", now / 1000);
			last_tick = now;
		}

		if (now - last_calc >= 2000) {
			uint32_t start = now;
			uint32_t n = sizeof(buf1) / sizeof(buf1[0]);

			printf("Number = %lu\n", n);

			for (int i = 0; i < n; ++i) {

				float angle = 2 * M_PI * i / n;
				float sin_value = sinf(angle);
				float adjusted = 0xffff / 2 + sin_value * 0xffff / 2;

				//printf("angle = %f sin(angle) = %f\n", angle, sin_value);

				buf1[i] = (uint16_t)adjusted;

			}

			printf("Calc took %lu us\n", GetTick() - start);

			last_calc = now;
		}

	}
}
