/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *  ESP-Drone Firmware
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
 * buzzer.c - Functions for interfacing with decks with buzzers
 */
#define DEBUG_MODULE "BUZZER"

#include "buzzer.h"
//#include "buzzdeck.h"
#include "sdkconfig.h"

static struct buzzerControl * ctrl;

extern void buzzDeckInit();

void buzzerInit()
{
#ifdef CONFIG_BUZZER_ON
    buzzDeckInit();
#endif
}

bool buzzerTest()
{
  return true;
}

void buzzerOff()
{
  if (ctrl)
    ctrl->off();
}

void buzzerOn(uint32_t freq)
{
  if (ctrl)
    ctrl->on(freq);
}

void buzzerSetControl(struct buzzerControl * bc)
{
  ctrl = bc;
}
