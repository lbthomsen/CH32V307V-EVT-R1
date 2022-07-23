/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
 双ADC混合同步规则+注入采样例程：
 规则组ADC1通道2(PA2)、ADC2通道4(PA4)，注入组ADC1通道3(PA3)ADC2通道5(PA5))
 规则组注入组均采用软件触发，通过DMA中断获取双ADC规则组数据，通过ADC中断获取双ADC注入组数据。
 */

#include <stdio.h>
#include <math.h>
#include "ch32v30x.h"
#include "systick.h"
#include "debug.h"

/* Global Variable */
u32 TxBuf[2 * 96];
u16 Adc_Val[2];
u16 ADC_Val1, ADC_Val2;
u8 Injected_IT_Flag, DMA_IT_Flag;
s16 Calibrattion_Val1 = 0;
s16 Calibrattion_Val2 = 0;
uint32_t gl_cnt = 0, ht_cnt = 0, tc_cnt = 0, te_cnt = 0, pa_cnt = 0;

void ADC1_2_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init (void) {
    ADC_InitTypeDef ADC_InitStructure = { 0 };
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC2, ENABLE);
    RCC_ADCCLKConfig (RCC_PCLK2_Div4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init (GPIOA, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    ADC_DeInit (ADC1);
    ADC_DeInit (ADC2);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Enable;
    ADC_InitStructure.ADC_Pga = ADC_Pga_1;

    ADC_Init (ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig (ADC1, ADC_Channel_2, 1, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig (ADC1, ADC_Channel_3, 2, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig (ADC1, ADC_Channel_3, 3, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig (ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);

    //ADC_InjectedSequencerLengthConfig (ADC1, 1);
    //ADC_InjectedChannelConfig (ADC1, ADC_Channel_3, 1, ADC_SampleTime_41Cycles5);

    //ADC_ExternalTrigInjectedConvConfig (ADC1, ADC_ExternalTrigConv_T8_TRGO);
    //ADC_ITConfig (ADC1, ADC_IT_JEOC, ENABLE);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    ADC_DMACmd (ADC1, ENABLE);
    ADC_Cmd (ADC1, ENABLE);

    ADC_BufferCmd (ADC1, DISABLE);   //disable buffer
    ADC_ResetCalibration (ADC1);
    while (ADC_GetResetCalibrationStatus (ADC1));
    ADC_StartCalibration (ADC1);
    while (ADC_GetCalibrationStatus (ADC1));

    Calibrattion_Val1 = Get_CalibrationValue (ADC1);

    ADC_BufferCmd (ADC1, ENABLE);   //enable buffer

    //ADC_Init (ADC2, &ADC_InitStructure);
    //ADC_RegularChannelConfig (ADC2, ADC_Channel_4, 1, ADC_SampleTime_41Cycles5);

    //ADC_InjectedSequencerLengthConfig (ADC2, 1);
    //ADC_InjectedChannelConfig (ADC2, ADC_Channel_5, 1, ADC_SampleTime_41Cycles5);

    //ADC_ExternalTrigInjectedConvConfig (ADC2, ADC_ExternalTrigConv_T8_TRGO);
    //ADC_ExternalTrigConvCmd (ADC2, ENABLE);
    //ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

    //ADC_ITConfig (ADC2, ADC_IT_JEOC, ENABLE);
    //ADC_Cmd (ADC2, ENABLE);

    //ADC_BufferCmd (ADC2, DISABLE);   //disable buffer
    //ADC_ResetCalibration (ADC2);
    //while (ADC_GetResetCalibrationStatus (ADC2));

    //ADC_StartCalibration (ADC2);
    //Calibrattion_Val2 = Get_CalibrationValue (ADC2);

    //while (ADC_GetCalibrationStatus (ADC2));

}

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init (DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit (DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init (DMA_CHx, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    DMA_ITConfig ( DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);
}

/*********************************************************************
 * @fn      Timer_Init
 *
 * @brief   Initializes Timers
 *
 * @return  none
 */
void Timer_Init (void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };

    //RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 3000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;


    //TIM_TimeBaseInit (TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit (TIM3, &TIM_TimeBaseStructure);

    //TIM_SelectOutputTrigger (TIM3, TIM_TRGOSource_Update);
    TIM_SelectOutputTrigger (TIM3, TIM_TRGOSource_Update);

    //TIM_Cmd (TIM3, ENABLE);
    TIM_Cmd (TIM3, ENABLE);

//    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);
//
//    TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
//    TIM_TimeBaseInit (TIM3, &TIM_TimeBaseStructure);
//
//    TIM_SelectOutputTrigger (TIM3, TIM_TRGOSource_Update);
//    TIM_Cmd (TIM3, ENABLE);

}

/*********************************************************************
 * @fn      Get_ConversionVal1
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal1 (s16 val) {
    if ((val + Calibrattion_Val1) < 0)
        return 0;
    if ((Calibrattion_Val1 + val) > 4095 || val == 4095)
        return 4095;
    return (val + Calibrattion_Val1);
}

/*********************************************************************
 * @fn      Get_ConversionVal2
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal2 (s16 val) {
    if ((val + Calibrattion_Val2) < 0)
        return 0;
    if ((Calibrattion_Val2 + val) > 4095 || val == 4095)
        return 4095;
    return (val + Calibrattion_Val2);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main (void) {

    Systick_Init ();
    Debug_Init (115200);

    //Delay_Init ();
    printf ("SystemClk:%d\r\n", SystemCoreClock);
    ADC_Function_Init ();
    printf ("ADC DMA test\r\n");
    printf ("CalibrattionValue1:%d\n", Calibrattion_Val1);
    printf ("CalibrattionValue2:%d\n", Calibrattion_Val2);

    DMA_Tx_Init ( DMA1_Channel1, (u32) &ADC1->RDATAR, (u32) TxBuf, 96);
    DMA_Cmd ( DMA1_Channel1, ENABLE);

    Timer_Init();

    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    uint32_t now = 0, last_run = 0;

    while (1) {

        now = GetTick();

        if (now - last_run >= 1000) {

            printf("\nTick %lu (tim3 cnt = %lu)\n", now/1000, TIM3->CNT);


            //ADC_ExternalTrigConvCmd(ADC2, ENABLE);
            //ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
            //ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);

            //ADC_SoftwareStartConvCmd (ADC1, ENABLE);
            //ADC_SoftwareStartConvCmd (ADC2, ENABLE);
            //ADC_SoftwareStartInjectedConvCmd (ADC1, ENABLE);
            //ADC_SoftwareStartInjectedConvCmd (ADC2, ENABLE);

            if (Injected_IT_Flag == 1) {
                Injected_IT_Flag = 0;
                printf ("JADC1 ch3=%04d\r\n", Get_ConversionVal1 (ADC_Val1));
                printf ("JADC2 ch5=%04d\r\n", Get_ConversionVal2 (ADC_Val2));
            }

                printf ("ADC1 ch2 = %d\r\n", Get_ConversionVal1 (Adc_Val[0]));
                printf ("ADC1 ch = %d\r\n", Get_ConversionVal2 (Adc_Val[1]));
                printf ("ADC2 ch4 = %d\r\n", Get_ConversionVal2 (Adc_Val[2]));
                printf ("DMA GL = %d HT = %d TC = %d TE = %d\n", gl_cnt, ht_cnt, tc_cnt, te_cnt);

            last_run = now;

        }

    }
}

/*********************************************************************
 * @fn      ADC1_2_IRQHandler
 *
 * @brief   This function handles ADC1_2 exception.
 *
 * @return  none
 */
void ADC1_2_IRQHandler () {
    if (ADC_GetITStatus ( ADC1, ADC_IT_JEOC)) {
        Injected_IT_Flag = 1;

        ADC_Val1 = ADC_GetInjectedConversionValue (ADC1, ADC_InjectedChannel_1);
        ADC_Val2 = ADC_GetInjectedConversionValue (ADC2, ADC_InjectedChannel_1);
    }
    ADC_ClearITPendingBit ( ADC1, ADC_IT_JEOC);
    ADC_ClearITPendingBit ( ADC2, ADC_IT_JEOC);
}

/*********************************************************************
 * @fn      DMA1_Channel1_IRQHandler
 *
 * @brief   This function handles DMA1 Channel1 exception.
 *
 * @return  none
 */
void DMA1_Channel1_IRQHandler () {

    if (DMA_GetITStatus(DMA1_IT_HT1) == SET) {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        ++ht_cnt;
    } else if (DMA_GetITStatus(DMA1_IT_TC1) == SET) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        ++tc_cnt;
    } else if (DMA_GetITStatus(DMA1_IT_TE1) == SET) {
        DMA_ClearITPendingBit(DMA1_IT_TE1);
        ++te_cnt;
    } else if (DMA_GetITStatus(DMA1_IT_GL1) == SET) {
        DMA_ClearITPendingBit (DMA1_IT_GL1);
        ++gl_cnt;
    } else {
        ++pa_cnt; // Panic
    }

//    if (DMA_GetITStatus (DMA1_IT_TC1) == SET) {
//        DMA_IT_Flag = 1;
//        DMA_ClearITPendingBit (DMA1_IT_GL1);
//
//        Adc_Val[0] = TxBuf[0] & 0xffff;
//        Adc_Val[1] = (TxBuf[0] >> 16) & 0xffff;
//
//        ++dma_cnt;
//    }
}

