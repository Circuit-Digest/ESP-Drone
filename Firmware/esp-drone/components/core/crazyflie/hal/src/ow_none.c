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
 * ow.c - One-wire functions
 */
#define DEBUG_MODULE "OW"

#include  <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "ow.h"


void owInit()
{
  owCommonInit();
}

bool owTest()
{
  return owCommonTest();
}

void owSyslinkRecieve(SyslinkPacket *slp)
{
}

bool owScan(uint8_t *nMem)
{
  return true;
}

bool owGetinfo(uint8_t selectMem, OwSerialNum *serialNum)
{
  return false;
}

bool owRead(uint8_t selectMem, uint16_t address, uint8_t length, uint8_t *data)
{
  return false;
}

bool owWrite(uint8_t selectMem, uint16_t address, uint8_t length, const uint8_t *data)
{
  return false;
}
