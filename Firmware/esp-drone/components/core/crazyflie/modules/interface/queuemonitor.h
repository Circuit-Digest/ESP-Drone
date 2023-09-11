/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * log.h - Dynamic log system
 */

#ifndef __QUEUE_MONITOR_H__
#define __QUEUE_MONITOR_H__


#include "FreeRTOS.h"

#ifdef DEBUG_QUEUE_MONITOR
  #include "queue.h"

  void queueMonitorInit();
  #define DEBUG_QUEUE_MONITOR_REGISTER(queue) qmRegisterQueue(queue, __FILE__, #queue)

  void qm_traceQUEUE_SEND(void* xQueue);
  void qm_traceQUEUE_SEND_FAILED(void* xQueue);
  void qmRegisterQueue(xQueueHandle* xQueue, char* fileName, char* queueName);
#else
  #define DEBUG_QUEUE_MONITOR_REGISTER(queue)
#endif // DEBUG_QUEUE_MONITOR

#endif // __QUEUE_MONITOR_H__