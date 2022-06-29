/*
 * systick.c
 *
 *  Created on: Jun 29, 2022
 *      Author: lth
 */

#include "ch32v30x.h"
#include "systick.h"

uint32_t uwTick = 0;

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles SysTick interrupt.
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void SysTick_Handler(void) {
	SysTick->SR=0;
	++uwTick;
}

/*********************************************************************
 * @fn      Systick_Init
 *
 * @brief   Initializes Systick.
 *
 * @return  none
 */
void Systick_Init(void) {
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
 * @fn      GetTick
 *
 * @brief   Get current tick in ms resolution
 *
 * @return  Current tick
 */
uint32_t GetTick() {
	return uwTick;
}

