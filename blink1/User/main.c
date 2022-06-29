/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : Lars Boegild Thomsen
* Version            : V1.0.0
* Date               : 2022/06/26
* Description        : Main program body.
* Copyright (c) 2022 Lars Boegild Thomsen <lbthomsen@gmail.com>
* SPDX-License-Identifier: MIT
*******************************************************************************/

#include "ch32v30x.h"

#include "debug.h"
#include "systick.h"

/* Global typedef */

/* Global define */

/* Global Variable */


/*********************************************************************
 * @fn      GPIO_Init
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIOS_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

	// Get the systick counter running
	Systick_Init();

	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf("SysTick Test\r\n");

	GPIOS_Init();

	uint32_t last_tick = 0;
	uint32_t last_toggle = 0;
	uint32_t loop = 0;
	uint8_t l0 = 0;
	uint8_t l1 = 1;

	while(1) {

		uint32_t now = GetTick();

		if (now - last_toggle >= 500) {

			GPIO_WriteBit(GPIOA, GPIO_Pin_0, (l0 == 0) ? (l0 = Bit_SET) : (l0 = Bit_RESET));
			GPIO_WriteBit(GPIOA, GPIO_Pin_1, (l1 == 0) ? (l1 = Bit_SET) : (l1 = Bit_RESET));

			last_toggle = now;
		}

		if (now - last_tick >= 1000) {
			printf("Tick = %lu - count = %lu\n", now, loop);

			loop = 0;
			last_tick = now;
		}

		++loop;

	}
}

