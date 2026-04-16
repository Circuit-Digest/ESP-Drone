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
 * console.h - Used to send console data to the client
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdbool.h>
#include "eprintf.h"

/**
 * Initialize the console
 */
void consoleInit(void);

bool consoleTest(void);

/**
 * Put a character to the console buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 */
int consolePutchar(int ch);

/**
 * Put a character to the console buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 *
 * @note This version can be called by interrup. In such case the internal
 * buffer is going to be used. If a task currently is printing or if the
 * interrupts prints too much the data will be ignored.
 */
int consolePutcharFromISR(int ch);

/**
 * Put a null-terminated string on the console buffer
 *
 * @param str Null terminated string
 * @return a nonnegative number on success, or EOF on error.
 */
int consolePuts(char *str);

/**
 * Flush the console buffer
 */
void consoleFlush(void);

/**
 * Macro implementing consolePrintf with eprintf
 *
 * @param FMT String format
 * @patam ... Parameters to print
 */
#define consolePrintf(FMT, ...) eprintf(consolePutchar, FMT, ## __VA_ARGS__)

#endif /*CONSOLE_H_*/
