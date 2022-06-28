/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : Lars Boegild Thomsen
* Version            : V1.0.0
* Date               : 2022/06/26
* Description        : Main program body.
* Copyright (c) 2022 Lars Boegild Thomsen <lbthomsen@gmail.com>
* SPDX-License-Identifier: MIT
*******************************************************************************/

#include "debug.h"


/* Global typedef */

/* Global define */

/* Global Variable */
uint32_t uwTick = 0;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      Systick_Init
 *
 * @brief   Initializes Systick.
 *
 * @return  none
 */
void Systick_Init(void)
{
    /*Configuration interrupt priority*/
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = SysTicK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//Seeing priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//Response priority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//Enable
    NVIC_Init(&NVIC_InitStructure);

    /*Configuration timer*/
    SysTick->CTLR= 0;
    SysTick->SR  = 0;
    SysTick->CNT = 0;
    SysTick->CMP = SystemCoreClock / 1000; //The latter 1000 represents 1000Hz (that is, 1MS to interrupt once)
    SysTick->CTLR= 0xf;
}

/*********************************************************************
 * @fn      GPIO_Init
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIOS_Init(void)
{
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
int main(void)
{
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf("SysTick Test\r\n");

	Systick_Init();
	GPIOS_Init();

	uint32_t last_tick = 0;
	uint32_t last_toggle = 0;
	uint32_t loop = 0;
	uint8_t l0 = 0;
	uint8_t l1 = 1;

	while(1)
    {

		uint32_t now = uwTick;

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

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles SysTick exception.
 *
 * @return  none
 */
void SysTick_Handler(void)
{
	SysTick->SR=0;
	++uwTick;
}

