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
 * configblock.h - Simple static implementation of the config block
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef __CONFIGBLOCK_H__
#define __CONFIGBLOCK_H__

int configblockInit(void);
bool configblockTest(void);

/* Static accessors */
int configblockGetRadioChannel(void);
int configblockGetRadioSpeed(void);
uint64_t configblockGetRadioAddress(void);

float configblockGetCalibPitch(void);
float configblockGetCalibRoll(void);

#endif //__CONFIGBLOCK_H__
