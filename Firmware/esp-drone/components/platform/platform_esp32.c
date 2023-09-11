/**
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
 * Platform functionality for platforms using the ESP32
 *
 */

#include <string.h>

#include "platform.h"

#define PLATFORM_INFO_OTP_NR_OF_BLOCKS 16
#define PLATFORM_INFO_OTP_BLOCK_LEN 32
#if PLATFORM_DEVICE_TYPE_STRING_MAX_LEN < (PLATFORM_INFO_OTP_BLOCK_LEN + 1)
#error
#endif

#define DEFAULT_PLATFORM_STRING "0;EP20"

//     //TODO:
// #ifndef UNIT_TEST_MODE
// static char *getAddressOfOtpMemoryBlock(int blockNr)
// {
//     //return block adress
//     return (char *)(0x1fff7800 + blockNr * 0x20);
// }
// #else
// // This function is replaced by a mock in unit tests
// char *getAddressOfOtpMemoryBlock(const int blockNr);
// #endif

void platformGetDeviceTypeString(char *deviceTypeString)
{
    char *block = 0;

    /*get platfrom type information from memory block  */

    //TODO:
    // for (int i = 0; i < PLATFORM_INFO_OTP_NR_OF_BLOCKS; i++) {
    //   char* candidateBlock = getAddressOfOtpMemoryBlock(i);
    //   if (candidateBlock[0] != 0) {
    //     block = candidateBlock;
    //     break;
    //   }
    // }

    /*use default platform information  */
    if (!block || ((unsigned char)block[0]) == 0xff) {
        block = DEFAULT_PLATFORM_STRING;
    }

    strncpy(deviceTypeString, block, PLATFORM_INFO_OTP_BLOCK_LEN);
    deviceTypeString[PLATFORM_INFO_OTP_BLOCK_LEN] = '\0';
}
