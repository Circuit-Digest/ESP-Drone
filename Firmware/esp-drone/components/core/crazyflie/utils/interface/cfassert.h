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
 * cfassert.h - Assert macro
 */

#include "console.h"

#ifndef __CFASSERT_H__
#define __CFASSERT_H__

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

#ifdef DEBUG_EP2
#define IF_DEBUG_ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )
#else
#define IF_DEBUG_ASSERT(e)
#endif

#define ASSERT_FAILED() assertFail( "", __FILE__, __LINE__ )

/**
 * @brief Assert that verifies that a pointer is pointing at memory that can be
 * used for DMA transfers. There are two types of RAM in the Crazyflie and CCM
 * does not work for DMA. Flash and RAM can be accessed by the DMA.
 *
 * @param[in] PTR : the pointer to verify
 */
#ifdef STM32F4XX
#define ASSERT_DMA_SAFE(PTR) if (((uint32_t)(PTR) >= 0x10000000) && ((uint32_t)(PTR) <=  0x1000FFFF)) assertFail( "", __FILE__, __LINE__ )
#else
#define ASSERT_DMA_SAFE(PTR)
#endif


/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);
/**
 * Print assert snapshot data
 */
void printAssertSnapshotData();
/**
 * Store assert snapshot data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertFileData(const char *file, int line);
/**
 * Store hardfault data to be read at startup if a reset is triggered (watchdog)
 * Line information can be printed using:
 * > make gdb
 * gdb> info line *0x<PC>
 */
void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr);

/**
 * Store assert data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertTextData(const char *text);

/**
 * @brief Check for assert information to indicate that the system was restarted
 * after a failed assert.
 *
 * @return true   If assert information exists
 * @return false  If no assert information exists
 */
bool cfAssertNormalStartTest(void);

#endif //__CFASSERT_H__
