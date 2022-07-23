/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : Lars Boegild Thomsen <lbthomsen@gmail.com>
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 * Copyright (c) 2022 Lars Boegild Thomsen
 * SPDX-License-Identifier: MIT
 *******************************************************************************/

#include <stdio.h>
#include "ch32v30x.h"
#include "systick.h"
#include "debug.h"

/* Global typedef */

/* Global define */
#define BUFFER_SIZE 48

/* Global Variable */
uint32_t adc_buffer[2 * BUFFER_SIZE * 6]; // 2 halves 6 channels

int16_t calibration_value;
uint32_t tc_cnt = 0, ht_cnt = 0;

/**
 * @fn          DMA1_Channel1_IRQHandler
 *
 * @brief       Service DMA1 Channel 1 Interrupt
 *
 * @return      none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void DMA1_Channel1_IRQHandler() {

    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        ++tc_cnt;
    } else if (DMA_GetITStatus(DMA1_IT_HT1) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        ++ ht_cnt;
    }

}

/**
 * @fn          Initialize_GPIO
 *
 * @brief       Initialize all GPIOs
 *
 * @return      none
 */
void Initialize_GPIO() {

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    // Make sure GPIOA is clocked
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);

    // Enable pa2, p3, p4, pa5 as analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init (GPIOA, &GPIO_InitStructure);

}

/**
 * @fn          Initialize_NVIC
 *
 * @brief       Initialize NVIC
 *
 * @return      none
 */
void Initialize_NVIC() {

    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    // DMA1 Channel 1 is used for the ADC1 DMA
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

}

/**
 * @fn          Initialize_ADC
 *
 * @brief       Initialize ADCs
 *
 * @return      none
 */
void Initialize_ADC() {

    ADC_InitTypeDef ADC_InitStructure = { 0 };

    // First let's give the ADC some clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Clock to ADC1 - both ADCs are hanging off of the APB2 bus
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); // What is PCLK2 - 144 MHz / 4 = 36 MHz?

    ADC_TempSensorVrefintCmd(ENABLE); // Let's enable the internal temperature sensor and vref

    ADC_DeInit(ADC1); // Start clean

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // No fancy injection stuff for now
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // Go through all
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // And just do it once
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO; // We will ASK when needed
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Enable;
    ADC_InitStructure.ADC_NbrOfChannel = 6;
    ADC_InitStructure.ADC_Pga = ADC_Pga_1;

    ADC_Init(ADC1, &ADC_InitStructure); // Kick it into gear

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 5, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 6, ADC_SampleTime_41Cycles5);

    ADC_Cmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    // Let's run a self calibration on the ADC
    ADC_BufferCmd(ADC1, DISABLE);   // disable buffer
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
    calibration_value = Get_CalibrationValue(ADC1);
    ADC_BufferCmd(ADC1, ENABLE);   // reenable buffer

}

/**
 * @fn          Initialize_DMA
 *
 * @brief       Initialize DMA
 *
 * @return      none
 */
void Initialize_DMA() {

    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit (DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_buffer[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 2 * BUFFER_SIZE * 6;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_ITConfig( DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);

}

/**
 * @fn          Initialize_Timers
 *
 * @brief       Initialize timers
 *
 * @return      none
 */
void Initialize_Timers() {

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };

    // Make sure the timer get clocked
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 3000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

    TIM_Cmd (TIM3, ENABLE);

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
    Debug_Init(115200);

    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    printf("SYSCLK = %lu\n", RCC_ClocksStatus.SYSCLK_Frequency);
    printf("PCLK1  = %lu\n", RCC_ClocksStatus.PCLK1_Frequency);
    printf("PCLK2  = %lu\n", RCC_ClocksStatus.PCLK2_Frequency);
    printf("HCLK   = %lu\n", RCC_ClocksStatus.HCLK_Frequency);
    printf("ADCCLK = %lu\n", RCC_ClocksStatus.ADCCLK_Frequency);

    printf("This is an ADC + DMA example\r\n");

    Initialize_GPIO();
    Initialize_NVIC();
    Initialize_ADC();
    Initialize_DMA();
    Initialize_Timers();

    uint32_t now = 0, last_tick = 0, loop_count = 0;

    while (1) {

        ++loop_count;

        now = GetTick();

        if (now - last_tick >= 1000) {

            printf("Tick %lu (count = %lu tc = %lu ht = %lu)\n", now / 1000, loop_count, tc_cnt, ht_cnt);

            loop_count = 0;
            last_tick = now;

        }

    }
}

/*
 * vim: ts=4 et nowrap
 */
