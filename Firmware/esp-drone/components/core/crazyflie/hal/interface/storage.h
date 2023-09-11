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
 * storage.h: Key/Buffer persistant storage
 *
 */


#pragma once

#include <stdbool.h>
#include <stddef.h>

/**
 * Initialize the storage subsystem.
 * 
 * This function is not reentrant.
 */
void storageInit();

/**
 * Test the storage subsystem
 * 
 * Check the key/buffer table health. If the table is not healthy, format it.
 */
bool storageTest();

/**
 * Store a buffer in a key. If the key already exist in the table,
 * it will be replaced.
 * 
 * This function can take a lot of time to complete: if there is no space for the new buffer,
 * the memory is going to be defragmented before the new buffer is written.
 * 
 * This function can fail either if there is no place left in memory or if the memory
 * is corrupted.
 * 
 * @param[key] Null terminated string for the key. Its length must be between 1 and 255.
 * @param[buffer] Pointer to the buffer to store
 * @param[length] Length of the buffer to store
 * 
 * @return true in case of success, false otherwise.
 */
bool storageStore(char* key, const void* buffer, size_t length);

/**
 * Fetch a buffer from the memory at some key.
 * 
 * @param[key] Null terminated string for the key. Its length must be between 1 and 255.
 * @param[buffer] Pointer to the buffer where the data should be copied
 * @param[length] Max length to copy in the buffer
 * 
 * @return length of the data read. This is the lowest length between the length of the
 *         receiving buffer, and the length of the data in memory. If the key is not found
 *         this function returns 0.
 */
size_t storageFetch(char *key, void* buffer, size_t length);

/**
 * Deletes and entry from the storage.
 * 
 * @param[key] Null terminated string for the key. Its length must be between 1 and 255.
 * 
 * @return true in case of success. false if the key was not found or if an error occured.
 */
bool storageDelete(char* key);
