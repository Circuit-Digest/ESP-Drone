/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * @file static_mem.h
 * @brief Utility macros to create static memory for OS objects such as
 * queues, semaphores and so on.
 *
 * @copyright Copyright (c) 2019
 *
 */

#pragma once

#include "cfassert.h"

#define CCM_NOT_SUPPORTED

/**
 * @brief Macro to indicate that a variable can be placed in the CCM
 * (Core Coupled Memroy) by the linker, instead of normal RAM.
 * The CCM has some special properties and can
 * not be used for DMA transfers, why special care should be taken to
 * make sure pointer to this memory will not be passed to a function
 * doing DMA.
 *
 * The memory is zero initialized at start up. If you assign a value
 * to the variable when declared, this value will be silently ignored
 * without a warning!
 *
 * Note: Using this macro does not guarantee that the variable will
 * end up in the CCM. The current implementation puts is in CCM but
 * that might change later.
 */
#if defined(UNIT_TEST_MODE)
  #define NO_DMA_CCM_SAFE_ZERO_INIT
#elif defined(CCM_NOT_SUPPORTED)
  #define NO_DMA_CCM_SAFE_ZERO_INIT
#else
  #define NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((section(".ccmbss")))
#endif

/**
 * @brief Macro to force a variable to be placed in the CCM
 * (Core Coupled Memroy) by the linker, instead of normal RAM.
 * The CCM has some special properties and can
 * not be used for DMA transfers, why special care should be taken to
 * make sure pointer to this memory will not be passed to a function
 * doing DMA.
 *
 * The memory is zero initialized at start up. If you assign a value
 * to the variable when declared, this value will be silently ignored
 * without a warning!
 */
#if defined(UNIT_TEST_MODE)
  #define FORCE_CCM_ZERO_INIT
#elif defined(CCM_NOT_SUPPORTED)
  #define FORCE_CCM_ZERO_INIT
#else
  #define FORCE_CCM_ZERO_INIT __attribute__((section(".ccmbss")))
#endif


/**
 * @brief Creation of queues using static memory.
 *
 * STATIC_MEM_QUEUE_ALLOC() and STATIC_MEM_QUEUE_CREATE() are used togehter to set up
 * and create a queue. STATIC_MEM_QUEUE_ALLOC() defines a number of static variables,
 * that are required by the queue. STATIC_MEM_QUEUE_CREATE() creates the OS queue object
 * using the previously defined variables.
 *
 * Example:
 * static xQueueHandle myQueue;
 * STATIC_MEM_QUEUE_ALLOC(myQueue, 5, sizeof(int));
 * // ...
 * void init() {
 *   myQueue = STATIC_MEM_QUEUE_CREATE(myQueue);
 *   // myQueue can now be used in OS calls
 * }
 *
 * Note: the queue handle may have the same name as used in the macros, but it is
 * not necessary.
 */

/**
 * @brief Creates a number of variables required for a queue using static memory.
 *
 * All variables created by this macreo are named using the NAME parameter as a base.
 *
 * Example:
 * STATIC_MEM_QUEUE_ALLOC(myQueue, 5, sizeof(int));
 *
 * expands to:
 * // static const int osSys_myQueueLength = 5;
 * // static const int osSys_myQueueItemSize = sizeof(int);
 * // static uint8_t osSys_myQueueStorage[5 * sizeof(int)];
 * // static StaticQueue_t osSys_myQueueSMgm;
 *
 * Note: the memory is allocated in CCM RAM. Read/write to the queue is done by copy
 * and the special properties of CCM RAM should not have any impact on the behaviour.
 *
 * @param NAME - the name of the queue handle (xQueueHandle). The name is also used as
 * base name for the other variables that are required.
 * @param LENGTH - the length of the queue (in items)
 * @param ITEM_SIZE - the size of the items in the queue
 */
#define STATIC_MEM_QUEUE_ALLOC(NAME, LENGTH, ITEM_SIZE)\
  static const int osSys_ ## NAME ## Length = (LENGTH); \
  static const int osSys_ ## NAME ## ItemSize = (ITEM_SIZE); \
  NO_DMA_CCM_SAFE_ZERO_INIT static uint8_t osSys_ ## NAME ## Storage[(LENGTH) * (ITEM_SIZE)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticQueue_t osSys_ ## NAME ## Mgm;

/**
 * @brief Creates a queue using static memory
 *
 * The queue is created under the assumption that variables have been created
 * using the STATIC_MEM_QUEUE_ALLOC() macro
 *
 * Example:
 * STATIC_MEM_QUEUE_CREATE(myQueue);
 * // Exoands to
 * // xQueueCreateStatic(osSys_myQueueLength, osSys_myQueueItemSize, osSys_myQueueStorage, &osSys_myQueueMgm);
 *
 * @param NAME - the name of the queue handle
 */
#define STATIC_MEM_QUEUE_CREATE(NAME) xQueueCreateStatic(osSys_ ## NAME ## Length, osSys_ ## NAME ## ItemSize, osSys_ ## NAME ## Storage, &osSys_ ## NAME ## Mgm)


/**
 * @brief Creation of tasks using static memory.
 *
 * STATIC_MEM_TASK_ALLOC() and STATIC_MEM_TASK_CREATE() are used together to
 * allocate buffers and create a task. STATIC_MEM_TASK_ALLOC() defines the
 * required memory and variables while STATIC_MEM_TASK_CREATE() creats
 * the task. The NAME is used as a base name for the necessary variables and
 * does not have to be the same as FUNCTION or TASK_NAME, but that works as well.
 *
 * Example:
 * static TaskHandle_t taskHandle;
 * STATIC_MEM_TASK_ALLOC(myTask, 100);
 *
 * void taskFcn(void *arg) {
 *   // Do stuff...
 * }
 *
 * void init() {
 *  taskHandle = STATIC_MEM_TASK_CREATE(myTask, taskFcn, "MY_TASK", NULL, 1);
 *  // ...
 * }
 */

/**
 * @brief Allocate variables and stack for a task using static memory.
 *
 * Note: the stack memory is allocated in normal RAM while the buffers
 * are allocated in CCM RAM. The special properties of CCM RAM should not
 * have any impact on the behaviour.

 * @param NAME A name used as base name for the variables that are created
 * @param STACK_DEPTH The stack depth in nr of StackType_t entries.
 */
#define STATIC_MEM_TASK_ALLOC(NAME, STACK_DEPTH) \
  static const int osSys_ ## NAME ## StackDepth = (STACK_DEPTH); \
  static StackType_t osSys_ ## NAME ## StackBuffer[(STACK_DEPTH)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t osSys_ ## NAME ## TaskBuffer;

/**
 * @brief Allocate variables and stack for a task using static memory.

 * All memory, including the stack is allocated in CCM RAM which means
 * that it can not be used for DMA transfers. Some drivers may use DMA
 * to retrieve data from a peripheral and it will NOT be possible to
 * pass a pointer from this stack to such driver
 *
 * @param NAME A name used as base name for the variables that are created
 * @param STACK_DEPTH The stack depth in nr of StackType_t entries.
 */
#define STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(NAME, STACK_DEPTH) \
  static const int osSys_ ## NAME ## StackDepth = (STACK_DEPTH); \
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t osSys_ ## NAME ## StackBuffer[(STACK_DEPTH)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t osSys_ ## NAME ## TaskBuffer;

/**
 * @brief Create a task using static memory
 *
 * The task is created under the assumption that STATIC_MEM_TASK_ALLOC() has been
 * used to define the required variables and buffers.
 *
 * @param NAME A name used as base name for the variables, same name that was used in STATIC_MEM_TASK_ALLOC()
 * @param FUNCTION The function that implements the task
 * @param TASK_NAME A descriptive name for the task
 * @param PARAMETERS Passed on as argument to the function implementing the task
 * @param PRIORITY The task priority
 */
#define STATIC_MEM_TASK_CREATE(NAME, FUNCTION, TASK_NAME, PARAMETERS, PRIORITY) xTaskCreateStatic((FUNCTION), (TASK_NAME), osSys_ ## NAME ## StackDepth, (PARAMETERS), (PRIORITY), osSys_ ## NAME ## StackBuffer, &osSys_ ## NAME ## TaskBuffer)
