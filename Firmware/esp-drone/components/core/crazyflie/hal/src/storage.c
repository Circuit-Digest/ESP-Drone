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
 * storage.c: Key/Buffer persistant storage
 *
 */

#include "storage.h"

#include "kve/kve.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "i2cdev.h"
#include "eeprom.h"

#define TRACE_MEMORY_ACCESS 0

#if !TRACE_MEMORY_ACCESS
#define DEBUG_MODULE "STORAGE"
#endif
#include "debug.h"

// Memory organization

// Low level memory access 

// ToDo: Shall we handle partition elsewhere?
#define KVE_PARTITION_START (1024)
#define KVE_PARTITION_LENGTH (7*1024)

static SemaphoreHandle_t storageMutex;

static size_t readEeprom(size_t address, void* data, size_t length)
{
  if (length == 0) {
    return 0;
  }

  bool success = eepromReadBuffer(data, KVE_PARTITION_START + address, length);

#if TRACE_MEMORY_ACCESS
  DEBUG_PRINT("R %s @%04x l%d: ", success?" OK ":"FAIL", address, length);

  for (int i=0; i < length; i++) {
    DEBUG_PRINT("%02x ", ((char*)data)[i]);
  }

  DEBUG_PRINT("\n");
#endif

  if (success) {
    return length;
  } else {
    return 0;
  }
}

static size_t writeEeprom(size_t address, const void* data, size_t length)
{
  if (length == 0) {
    return 0;
  }

  bool success = eepromWriteBuffer(data, KVE_PARTITION_START + address, length);

#if TRACE_MEMORY_ACCESS
  DEBUG_PRINT("W %s @%04x: ", success?" OK ":"FAIL", address);

  for (int i=0; i < length; i++) {
    DEBUG_PRINT("%02x ", ((char*)data)[i]);
  }

  DEBUG_PRINT("\n");
#endif

  if (success) {
    return length;
  } else {
    return 0;
  }
}

static void flushEeprom(void)
{
  // NOP for now, lets fix the EEPROM write first!
}

static kveMemory_t kve = {
  .memorySize = KVE_PARTITION_LENGTH,
  .read = readEeprom,
  .write = writeEeprom,
  .flush = flushEeprom,
};

// Public API

static bool isInit = false;

void storageInit()
{
  storageMutex = xSemaphoreCreateMutex();

  isInit = true;
}

bool storageTest()
{
  bool pass = kveCheck(&kve);

  DEBUG_PRINT("Storage check %s.\n", pass?"[OK]":"[FAIL]");

  if (!pass) {
    DEBUG_PRINT("Reformating storage ...\n");

    kveFormat(&kve);

    pass = kveCheck(&kve);
    DEBUG_PRINT("Storage check %s.\n", pass?"[OK]":"[FAIL]");

    if (pass == false) {
      DEBUG_PRINT("Error: Cannot format storage!\n");
    }
  }

  return pass;
}

bool storageStore(char* key, const void* buffer, size_t length)
{
  if (!isInit) {
    return false;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  bool result = kveStore(&kve, key, buffer, length);

  xSemaphoreGive(storageMutex);

  return result;
}

size_t storageFetch(char *key, void* buffer, size_t length)
{
  if (!isInit) {
    return 0;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  size_t result = kveFetch(&kve, key, buffer, length);

  xSemaphoreGive(storageMutex);

  return result;
}

bool storageDelete(char* key)
{
  if (!isInit) {
    return false;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  bool result = kveDelete(&kve, key);

  xSemaphoreGive(storageMutex);

  return result;
}
