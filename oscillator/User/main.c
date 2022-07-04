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


/* Global define */
#define MAX_VALUE 4095
#define MID_POINT 2047
#define SAMPLE_FREQ 48000
#define BUFFER_SIZE 48 // 48 kHz every 1 ms

/* Global typedef */

typedef struct {
	float angle;
	float angle_per_sample;
	float amplitude;
	float last_value;
} Osc_TypeDef;

/* Global Variable */

Osc_TypeDef osc[2]; // Two oscillators

uint32_t dac_buffer[2 * BUFFER_SIZE]; // High 16 bit dac out 2, low 16 bit dac out 1

uint32_t full_count = 0, half_count = 0;

static inline void update_dac_buffer(uint32_t buffer_address) {
	for (uint8_t sample = 0; sample < BUFFER_SIZE; ++sample) {
		for (uint8_t oscillator = 0; oscillator < 2; ++oscillator) {
			osc[oscillator].last_value = osc[oscillator].amplitude * sinf(osc[oscillator].angle);
			osc[oscillator].angle += osc[oscillator].angle_per_sample; // rotate
			if (osc[oscillator].angle > M_TWOPI) osc[oscillator].angle -= M_TWOPI; // roll over
		}
		//dac_buffer[sample] = ((uint16_t)(MID_POINT + MID_POINT * osc[1].last_value)) << 16 | ((uint16_t)osc[0].last_value);
		//dac_buffer[sample] = ((uint16_t)(MID_POINT + MID_POINT * osc[0].last_value));
		dac_buffer[sample] = (((uint16_t)(MID_POINT + MID_POINT * osc[1].last_value)) << 16) |  ((uint16_t)(MID_POINT + MID_POINT * osc[0].last_value));
	}
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void DMA2_Channel3_IRQHandler() {

	GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
	if (DMA_GetITStatus(DMA2_IT_TC3) != RESET) {
		++full_count;
		update_dac_buffer(dac_buffer[BUFFER_SIZE]);
		DMA_ClearITPendingBit(DMA2_IT_TC3);
	} else if (DMA_GetITStatus(DMA2_IT_HT3) != RESET) {
		++half_count;
		update_dac_buffer(dac_buffer[0]);
		DMA_ClearITPendingBit(DMA2_IT_HT3);
	}
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
}

void Dac_Interrupt_Init() {
	/*Configuration interrupt priority*/
	NVIC_InitTypeDef NVIC_InitStructure = { 0 };
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //Seeing priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Response priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Enable
	NVIC_Init(&NVIC_InitStructure);
}

/*********************************************************************
 * @fn      Dac_Init
 *
 * @brief   Initializes DAC collection.
 *
 * @return  none
 */
void Dual_Dac_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };
	DAC_InitTypeDef DAC_InitType = { 0 };

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Debug out
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;

	DAC_InitType.DAC_Trigger = DAC_Trigger_T4_TRGO;
	DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitType);
	DAC_Init(DAC_Channel_2, &DAC_InitType);

	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);

	DAC_DMACmd(DAC_Channel_1, ENABLE);
	DAC_DMACmd(DAC_Channel_2, ENABLE);

	DAC_SetDualChannelData(DAC_Align_12b_R, 0x123, 0x321);
}

/*********************************************************************
 * @fn      DAC1_DMA_INIT
 *
 * @brief   Initializes DMA of DAC1 collection.
 *
 * @return  none
 */
void Dac_Dma_Init(void) {
	DMA_InitTypeDef DMA_InitStructure = { 0 };
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(DAC->RD12BDHR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &dac_buffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA2_Channel3, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Channel3, DMA_IT_HT, ENABLE);

	DMA_Cmd(DMA2_Channel3, ENABLE);

}

/*********************************************************************
 * @fn      Timer4_Init
 *
 * @brief   Initializes TIM4
 *
 * @return  none
 */
void Timer4_Init(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 3000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);
	TIM_Cmd(TIM4, ENABLE);
}


void set_freq(Osc_TypeDef *osc, float freq) {
	osc->angle_per_sample = M_TWOPI / (SAMPLE_FREQ / freq);
}

void set_amplitude(Osc_TypeDef *osc, float amplitude) {
	osc->amplitude = amplitude;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

	Systick_Init();

	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n", SystemCoreClock);
	printf("Dual DAC Generation Test\r\n");

	set_freq(&osc[0], 110); set_amplitude(&osc[0], 0.8);
	set_freq(&osc[1], 220); set_amplitude(&osc[1], 0.4);

	Dac_Interrupt_Init();
	Dual_Dac_Init();
	Dac_Dma_Init();
	Timer4_Init();

	uint32_t now = 0, last_tick = 0;

	while (1) {

		now = GetTick();

		if (now - last_tick >= 1000) {

			printf("Full Count = %lu Half Count = %lu\n", full_count, half_count);

			last_tick = now;
		}

	}
}
