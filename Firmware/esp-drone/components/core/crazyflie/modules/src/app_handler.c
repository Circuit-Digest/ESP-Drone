/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2019, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* app_handler.c: App layer handling function implementation */

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "static_mem.h"

#include "app.h"

#ifndef APP_STACKSIZE
#define APP_STACKSIZE 300
#endif

#ifndef APP_PRIORITY
#define APP_PRIORITY 0
#endif

static bool isInit = false;

STATIC_MEM_TASK_ALLOC(appTask, APP_STACKSIZE);

static void appTask(void *param);

void __attribute__((weak)) appInit()
{
  if (isInit) {
    return;
  }

  STATIC_MEM_TASK_CREATE(appTask, appTask, "app", NULL, APP_PRIORITY);
  isInit = true;
}

static void appTask(void *param)
{
  systemWaitStart();

  appMain();

  while(1) {
    vTaskDelay(portMAX_DELAY);
  }
}
