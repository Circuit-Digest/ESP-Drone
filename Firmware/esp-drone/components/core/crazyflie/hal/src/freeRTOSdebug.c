/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * debug.c - Various debug functions
 */
#include <stdint.h>

#include "freertos/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#define DEBUG_MODULE "FreeRTOSConfig"
#include "debug_cf.h"
#include "nvicconf.h"
#include "led.h"
#include "power_distribution.h"

uint32_t traceTickCount;

//TODO:disable system hook now
// void vApplicationMallocFailedHook( void )
// {
//   portDISABLE_INTERRUPTS();
//   DEBUG_PRINTE("Malloc failed!\n");
//   ledSet(ERR_LED1, 1);
//   ledSet(ERR_LED2, 1);
//   powerStop();
//   storeAssertTextData("Malloc failed");
//   while(1);
// }

// #if (configCHECK_FOR_STACK_OVERFLOW > 0)
// void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
// {
//   portDISABLE_INTERRUPTS();
//   DEBUG_PRINT("\nStack overflow!\n");
//   ledSet(ERR_LED1, 1);
//   ledSet(ERR_LED2, 1);
//   powerStop();
//   storeAssertTextData("Stack overflow");
//   while(1);
// }
// #endif

#ifdef UART_OUTPUT_TRACE_DATA
void debugSendTraceInfo(unsigned int taskNbr)
{
  uint32_t traceData;
  traceData = (taskNbr << 29) | (((traceTickCount << 16) + TIM1->CNT) & 0x1FFFFFF);
  uartSendDataDma(sizeof(traceData), (uint8_t*)&traceData);
}

void debugInitTrace(void)
{
  /*TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //Enable the Timer
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 72;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TRACE_TIM_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM1, ENABLE);

  traceTickCount = 0;*/
}
#else
void debugSendTraceInfo(unsigned int taskNbr)
{
}
#endif
