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
 * sleepus.h: Micro second sleep
 */
#include <stdint.h>
#include <stdbool.h>

#include "sleepus.h"

#include "config.h"

//#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usec_time.h"

#define TICK_PER_US (FREERTOS_MCU_CLOCK_HZ / (8 * 1e6))

static bool isInit = false;

void sleepus(uint32_t us)
{
  if (!isInit) {
    initUsecTimer();
    isInit = true;
  }

  uint64_t start = usecTimestamp();

  while ((start+us) > usecTimestamp());
}
