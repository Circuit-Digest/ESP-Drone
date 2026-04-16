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
 * sound.h - Module used to play melodies and system sounds though a buzzer
 */

#ifndef __SOUND_H__
#define __SOUND_H__

#include <stdint.h>
#include <stdbool.h>

#define SND_OFF         0
#define FACTORY_TEST    1
#define SND_USB_DISC    2
#define SND_USB_CONN    3
#define SND_BAT_FULL    4
#define SND_BAT_LOW     5
#define SND_STARTUP     6
#define SND_CALIB       7

/**
 * Initialize sound sub-system.
 */
void soundInit(void);

/**
 * Test the sound sub-system.
 */
bool soundTest(void);

/**
 * Set the effect that should be played by the sound sub-system.
 */
void soundSetEffect(uint32_t effect);

/**
 * Manufally set a frequency that should be played by the sound syb-system.
 */
void soundSetFreq(uint32_t freq);

#endif /* __SOUND_H__ */

