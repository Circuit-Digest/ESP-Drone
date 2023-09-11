/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 * worker.c - Worker system that can execute asynchronous actions in tasks
 */
#include "worker.h"

#include <errno.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "static_mem.h"

#include "console.h"

#define WORKER_QUEUE_LENGTH 5

struct worker_work {
  void (*function)(void*);
  void* arg;
};

static xQueueHandle workerQueue;
STATIC_MEM_QUEUE_ALLOC(workerQueue, WORKER_QUEUE_LENGTH, sizeof(struct worker_work));

void workerInit()
{
  if (workerQueue)
    return;

  workerQueue = STATIC_MEM_QUEUE_CREATE(workerQueue);
  DEBUG_QUEUE_MONITOR_REGISTER(workerQueue);
}

bool workerTest()
{
  return (workerQueue != NULL);
}

void workerLoop()
{
  struct worker_work work;

  if (!workerQueue)
    return;

  while (1)
  {
    xQueueReceive(workerQueue, &work, portMAX_DELAY);

    if (work.function)
      work.function(work.arg);
  }
}

int workerSchedule(void (*function)(void*), void *arg)
{
  struct worker_work work;

  if (!function)
    return ENOEXEC;

  work.function = function;
  work.arg = arg;
  if (xQueueSend(workerQueue, &work, 0) == pdFALSE)
    return ENOMEM;

  return 0;
}
