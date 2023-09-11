/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 *
 * kveStorage.h - Low level storage functions
 *
 */

/**
 * These functions are intended to be used insternally by the embedded
 * key-value module. They are not intended to be used by any other
 * modules.
 */

#pragma once

#include "kve/kve_common.h"

#include <stddef.h>
#include <stdint.h>


#define KVE_STORAGE_IS_VALID(a) (a != SIZE_MAX)

#define KVE_STORAGE_INVALID_ADDRESS (SIZE_MAX)

#define END_TAG_LENDTH 2

typedef struct itemHeader_s {
  uint16_t full_length;
  uint8_t key_length;
} __attribute((packed)) kveItemHeader_t;

/** Add the item at address "address"
 * 
 * This is a utility function that does not check for anything, the caller is
 * responsible for checking that no access after the end of the memory is made
 * 
 * Return the full length of the item in memory
 */
int kveStorageWriteItem(kveMemory_t *kve, size_t address, const char* key, const void* buffer, size_t length);

/** Write holes spanning full_length at address
 * 
 * The hole MUST be at least 3 bytes wide. No check is done in this function!
 * 
 * Return the full length of the item in memory
 */
uint16_t kveStorageWriteHole(kveMemory_t *kve, size_t address, size_t full_length);

/** Write end the the table at address
 * 
 * No check is done in this function, there should be
 * at least 2 bytes available at address
 * 
 * Return the full length of the item in memory
 */
uint16_t kveStorageWriteEnd(kveMemory_t *kve, size_t address);

/** Move a block of memory from an address to another
 * 
 * Source address MUST be > than destination address
 */
void kveStorageMoveMemory(kveMemory_t *kve, size_t sourceAddress, size_t destinationAddress, size_t length);

size_t kveStorageFindItemByKey(kveMemory_t *kve, size_t address, const char * key);

/** Find and return the address of the end of table
 * 
 * Address can be set to the begining of an item to start the search
 * in the middle of the table.
 * 
 * This function return an invalid address if the end of the memory
 * is reached before finding the end tag. If that appens, the table
 * is corrupted! (good for checking the health of the kve table)
 */
size_t kveStorageFindEnd(kveMemory_t *kve, size_t address);

size_t kveStorageFindHole(kveMemory_t *kve, size_t address);

size_t kveStorageFindNextItem(kveMemory_t *kve, size_t address);

kveItemHeader_t kveStorageGetItemInfo(kveMemory_t *kve, size_t address);

size_t kveStorageGetKeyLength(kveItemHeader_t header);

size_t kveStorageGetBufferLength(kveItemHeader_t header);

size_t kveStorageGetKey(kveMemory_t *kve, size_t address, kveItemHeader_t header, char* key, size_t maxLength);

size_t kveStorageGetBuffer(kveMemory_t *kve, size_t address, kveItemHeader_t header, void* buffer, size_t maxLength);