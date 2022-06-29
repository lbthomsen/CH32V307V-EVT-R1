/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.h
* Author             : Lars Boegild Thomsen <lbthomsen@gmail.com>
* Version            : V1.0.0
* Date               : 2022/06/26
* Description        : This file contains all the functions prototypes for UART
*                      Printf functions.
* Copyright (c) 2022 Lars Boegild Thomsen
* SPDX-License-Identifier: MIT
*******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#include "stdio.h"
#include "ch32v30x.h"

/* UART Printf Definition */
#define DEBUG_UART1    1
#define DEBUG_UART2    2
#define DEBUG_UART3    3

/* DEBUG UATR Definition */
#define DEBUG   DEBUG_UART1
//#define DEBUG   DEBUG_UART2
//#define DEBUG   DEBUG_UART3

void USART_Printf_Init(uint32_t baudrate);

#endif 
