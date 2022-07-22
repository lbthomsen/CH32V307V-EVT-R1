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

/* Global Variable */

int16_t calibration_value;

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

    ADC_DeInit(ADC1); // Start clean

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // No fancy injection stuff for now
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // Just sample one
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // And just do it once
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // We will ASK when needed
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure); // Kick it into gear

    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, DISABLE);   // disable buffer
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
    calibration_value = Get_CalibrationValue(ADC1);

    ADC_BufferCmd(ADC1, ENABLE);   // reenable buffer

    ADC_TempSensorVrefintCmd(ENABLE);

}

/*********************************************************************
 * @fn      Get_ADC_Val
 *
 * @brief   Returns ADCx conversion result data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *            ADC_Channel_10 - ADC Channel10 selected.
 *            ADC_Channel_11 - ADC Channel11 selected.
 *            ADC_Channel_12 - ADC Channel12 selected.
 *            ADC_Channel_13 - ADC Channel13 selected.
 *            ADC_Channel_14 - ADC Channel14 selected.
 *            ADC_Channel_15 - ADC Channel15 selected.
 *            ADC_Channel_16 - ADC Channel16 selected.
 *            ADC_Channel_17 - ADC Channel17 selected.
 *
 * @return  none
 */
u16 Get_ADC_Val(u8 ch) {
    u16 val;

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_28Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    uint32_t cnt = 0;
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) ++cnt;

    //printf("cnt = %lu\n", cnt);

    val = ADC_GetConversionValue(ADC1);

    return val;
}

/*********************************************************************
 * @fn      Get_ADC_Average
 *
 * @brief   Returns ADCx conversion result average data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *            ADC_Channel_10 - ADC Channel10 selected.
 *            ADC_Channel_11 - ADC Channel11 selected.
 *            ADC_Channel_12 - ADC Channel12 selected.
 *            ADC_Channel_13 - ADC Channel13 selected.
 *            ADC_Channel_14 - ADC Channel14 selected.
 *            ADC_Channel_15 - ADC Channel15 selected.
 *            ADC_Channel_16 - ADC Channel16 selected.
 *            ADC_Channel_17 - ADC Channel17 selected.
 *
 * @return  val - The Data conversion value.
 */
u16 Get_ADC_Average(u8 ch, u8 times) {
    u32 temp_val = 0;
    u8 t;
    u16 val;

    for (t = 0; t < times; t++) {
        temp_val += Get_ADC_Val(ch);
    }

    val = temp_val / times;

    return val;
}

/*********************************************************************
 * @fn      Get_ConversionVal
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal(s16 val) {
    if ((val + calibration_value) < 0)
        return 0;
    if ((calibration_value + val) > 4095 || val == 4095)
        return 4095;
    return (val + calibration_value);
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

    printf("This is an ADC example\r\n");

    Initialize_ADC();

    uint32_t now = 0, last_tick = 0, loop_count = 0;

    while (1) {

        ++loop_count;

        now = GetTick();

        if (now - last_tick >= 1000) {

            uint16_t vref_voltage = Get_ConversionVal(Get_ADC_Average( ADC_Channel_Vrefint, 10)) * 3300 / 4096;
            uint16_t temp_voltage = Get_ConversionVal(Get_ADC_Average( ADC_Channel_TempSensor, 10)) * 3300 / 4096;
            int32_t temp_value = TempSensor_Volt_To_Temper(temp_voltage);

            printf("Tick %lu (count = %lu) vt = %lu temp = %lu vref = %lu\n", now / 1000, loop_count, temp_voltage, temp_value, vref_voltage);

            loop_count = 0;
            last_tick = now;

        }

    }
}

/*
 * vim: ts=4 et nowrap
 */
