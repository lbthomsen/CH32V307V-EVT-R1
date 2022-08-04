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
 串口打印调试例程：
 USART1_Tx(PA9)。
 本例程演示使用 USART1(PA9) 作打印调试口输出。

 */

#include "ch32v30x.h"

#include "debug.h"
#include "systick.h"

/* Global typedef */

/* Global define */

/* Global Variable */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

#define BOOTLOADER_ADDRESS 0x1FFF8000

    typedef void (*pFunction)(void);
    pFunction JumpToApplication;
    uint32_t JumpAddress;

    /* Jump to system memory bootloader */
    JumpAddress = *(__IO uint32_t*) (BOOTLOADER_ADDRESS + 4);
    JumpToApplication = (pFunction) JumpAddress;
    JumpToApplication();

    // Get the systick counter running
    Systick_Init();

    Debug_Init(115200);

    printf("SystemClk:%d\r\n", SystemCoreClock);

    printf("This is printf example\r\n");

    while (1) {

    }
}

