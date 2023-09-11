/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * buzzdeck.h - Functions for interfacing with decks with buzzers
 */
#ifndef __BUZZER_H__
#define __BUZZER_H__
 
#include <stdint.h>
#include <stdbool.h>

/** Functionpointers used to control the buzzer */
struct buzzerControl
{
  void (*off)();
  void (*on)(uint32_t freq);
};

/**
 * Initilize the buzzer sub-system.
 */
void buzzerInit();

/**
 * Test the buzzer sub-system.
 */
bool buzzerTest();

/**
 * Turn the buzzer off.
 */
void buzzerOff();

/**
 * Turn the buzzer on and set it to a specific frequency (if supported).
 */
void buzzerOn(uint32_t freq);

/**
 * Set function pointers for controlling the buzzer hardware.
 */
void buzzerSetControl(struct buzzerControl * bc);

#endif //__BUZZER_H__

