/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2020 BitCraze AB
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
 * mem.h - Memory sub system
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  MEM_TYPE_EEPROM = 0x00,
  MEM_TYPE_OW     = 0x01,
  MEM_TYPE_LED12  = 0x10,
  MEM_TYPE_LOCO   = 0x11,
  MEM_TYPE_TRAJ   = 0x12,
  MEM_TYPE_LOCO2  = 0x13,
  MEM_TYPE_LH     = 0x14,
  MEM_TYPE_TESTER = 0x15,
  MEM_TYPE_USD    = 0x16,
  MEM_TYPE_LEDMEM = 0x17,
  MEM_TYPE_APP    = 0x18,
} MemoryType_t;

#define MEMORY_SERIAL_LENGTH 8

typedef struct {
  const MemoryType_t type;
  uint32_t (*getSize)(void);
  bool (*read)(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
  bool (*write)(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
} MemoryHandlerDef_t;

typedef struct {
  uint8_t nrOfMems;
  const uint32_t size;
  bool (*getSerialNr)(const uint8_t selectedMem, uint8_t* serialNr);
  bool (*read)(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
  bool (*write)(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
} MemoryOwHandlerDef_t;

/* Public functions */
void memInit(void);
bool memTest(void);
void memoryRegisterHandler(const MemoryHandlerDef_t* handlerDef);
void memoryRegisterOwHandler(const MemoryOwHandlerDef_t* handlerDef);
