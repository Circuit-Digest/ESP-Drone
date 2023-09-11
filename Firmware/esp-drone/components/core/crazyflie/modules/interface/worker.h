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
 * worker.h - Worker system that can execute asynchronous actions in tasks
 */
#ifndef __WORKER_H
#define __WORKER_H

#include <stdbool.h>

void workerInit();

bool workerTest();

/**
 * Light printf implementation
 *
 * This function exectute the worker loop and never returns except if the worker
 * module has not been initialized.
 */
void workerLoop();

/**
 * Schedule a function for execution by the worker loop
 * The function will be executed as soon as possible by the worker loop.
 * Scheduled functions are stacked in a FIFO queue.
 *
 * @param function Function to be executed
 * @param arg      Argument that will be passed to the function when executed
 * @return         0 in case of success. Anything else on failure.
 */
int workerSchedule(void (*function)(void*), void *arg);

#endif //__WORKER_H
