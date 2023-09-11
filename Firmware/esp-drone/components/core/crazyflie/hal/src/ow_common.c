/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 * ow_common.c - One-wire functions
 */

#include "ow.h"
#include "mem.h"

static bool handleMemGetSerialNr(const uint8_t selectedMem, uint8_t* serialNr);
static bool handleMemRead(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData);
static bool handleMemWrite(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData);

static MemoryOwHandlerDef_t memHandlerDef = {
  .nrOfMems = 0,
  .size = OW_MAX_SIZE,
  .getSerialNr = handleMemGetSerialNr,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static bool isInit = false;

void owCommonInit() {
  if (isInit) {
    return;
  }

  owScan(&memHandlerDef.nrOfMems);
  memoryRegisterOwHandler(&memHandlerDef);

  isInit = true;
}

bool owCommonTest() {
  return isInit;
}

static bool handleMemGetSerialNr(const uint8_t selectedMem, uint8_t* serialNr) {
  return owGetinfo(selectedMem, (OwSerialNum*)serialNr);
}

static bool handleMemRead(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData) {
  bool result = false;

  if (memAddr + readLen <= OW_MAX_SIZE) {
    if (owRead(selectedMem, memAddr, readLen, startOfData)) {
      result = true;
    }
  }

  return result;
}

static bool handleMemWrite(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData) {
  bool result = false;

  if (memAddr + writeLen <= OW_MAX_SIZE) {
    if (owWrite(selectedMem, memAddr, writeLen, startOfData)) {
      result = true;
    }
  }

  return result;
}
