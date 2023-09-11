/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2019 BitCraze AB
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
 * mem.c: Memory module. Handles one-wire, eeprom and other memory mapping memory functions over crtp link.
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mem.h"
#include "crtp.h"
#include "system.h"

#include "console.h"
#include "assert.h"
#define DEBUG_MODULE "MEM_CF2"
#include "debug_cf.h"

#include "log.h"
#include "param.h"
#include "static_mem.h"

#if 0
#define MEM_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)
#define MEM_ERROR(fmt, ...) DEBUG_PRINT("E/log " fmt, ## __VA_ARGS__)
#else
#define MEM_DEBUG(...)
#define MEM_ERROR(...)
#endif


// Maximum payload length
#define MEM_MAX_LEN 30

#define MEM_SETTINGS_CH     0
#define MEM_READ_CH         1
#define MEM_WRITE_CH        2

#define MEM_CMD_GET_NBR     1
#define MEM_CMD_GET_INFO    2

#define STATUS_OK 0

#define MEM_TESTER_SIZE            0x1000

//Private functions
static void memTask(void * prm);
static void memSettingsProcess(CRTPPacket* p);
static void memWriteProcess(CRTPPacket* p);
static void memReadProcess(CRTPPacket* p);
static void createNbrResponse(CRTPPacket* p);
static void createInfoResponse(CRTPPacket* p, uint8_t memId);
static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]);

static uint32_t handleMemTesterGetSize(void) { return MEM_TESTER_SIZE; }
static bool handleMemTesterRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemTesterWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData);
static uint32_t memTesterWriteErrorCount = 0;
static uint8_t memTesterWriteReset = 0;
static const MemoryHandlerDef_t memTesterDef = {
  .type = MEM_TYPE_TESTER,
  .getSize = handleMemTesterGetSize,
  .read = handleMemTesterRead,
  .write = handleMemTesterWrite,
};

static bool isInit = false;
static bool registrationEnabled = true;

static uint8_t nbrOwMems = 0;
static const uint8_t NoSerialNr[MEMORY_SERIAL_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0};
static CRTPPacket packet;

#define MAX_NR_HANDLERS 20
static const MemoryHandlerDef_t* handlers[MAX_NR_HANDLERS];
static uint8_t nrOfHandlers = 0;
static const MemoryOwHandlerDef_t* owMemHandler = 0;

STATIC_MEM_TASK_ALLOC(memTask, MEM_TASK_STACKSIZE);

void memInit(void)
{
  if(isInit) {
    return;
  }

  memoryRegisterHandler(&memTesterDef);

  //Start the mem task
  STATIC_MEM_TASK_CREATE(memTask, memTask, MEM_TASK_NAME, NULL, MEM_TASK_PRI);

  isInit = true;
}

bool memTest(void) {
  if (!isInit) {
    return false;
  }

  if (owMemHandler == 0) {
    return false;
  }

  return true;
}

void memoryRegisterHandler(const MemoryHandlerDef_t* handlerDef){
  for (int i = 0; i < nrOfHandlers; i++) {
    ASSERT(handlerDef->type != handlers[i]->type);
  }
  ASSERT(nrOfHandlers < MAX_NR_HANDLERS);
  ASSERT(registrationEnabled);
  handlers[nrOfHandlers] = handlerDef;
  nrOfHandlers++;
}

void memoryRegisterOwHandler(const MemoryOwHandlerDef_t* handlerDef){
  ASSERT(owMemHandler == 0);
  owMemHandler = handlerDef;

  nbrOwMems = handlerDef->nrOfMems;
}

static void memTask(void* param) {
	crtpInitTaskQueue(CRTP_PORT_MEM);

  systemWaitStart();

  // Do not allow registration of new handlers after this point as clients now can start
  // to query for available memories
  registrationEnabled = false;

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_MEM, &packet);

		switch (packet.channel) {
      case MEM_SETTINGS_CH:
        memSettingsProcess(&packet);
        break;
      case MEM_READ_CH:
        memReadProcess(&packet);
        break;
      case MEM_WRITE_CH:
        memWriteProcess(&packet);
        break;
      default:
        // Do nothing
        break;
		}
	}
}

static void memSettingsProcess(CRTPPacket* p) {
  switch (p->data[0]) {
    case MEM_CMD_GET_NBR:
      createNbrResponse(p);
      crtpSendPacket(p);
      break;

    case MEM_CMD_GET_INFO:
      {
        uint8_t memId = p->data[1];
        createInfoResponse(p, memId);
        crtpSendPacket(p);
      }
      break;

    default:
      // Do nothing
      break;
  }
}

static void createNbrResponse(CRTPPacket* p) {
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_NBR;
  p->data[1] = 0;//nbrOwMems + OW_FIRST_ID;
}

static void createInfoResponse(CRTPPacket* p, uint8_t memId) {
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_INFO;
  p->data[1] = memId;

  if (memId < nrOfHandlers) {
    createInfoResponseBody(p, handlers[memId]->type, handlers[memId]->getSize(), NoSerialNr);
  } else {
    const uint8_t selectedMem = memId - nrOfHandlers;
    uint8_t serialNr[MEMORY_SERIAL_LENGTH];

    // No error code if we fail, just send an empty packet back
    if (owMemHandler->getSerialNr(selectedMem, serialNr)) {
      createInfoResponseBody(p, MEM_TYPE_OW, owMemHandler->size, serialNr);
    }
  }
}

static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]) {
  p->data[2] = type;
  p->size += 1;

  memcpy(&p->data[3], &memSize, 4);
  p->size += 4;

  memcpy(&p->data[7], data, 8);
  p->size += 8;
}


static void memReadProcess(CRTPPacket* p) {
  uint32_t memAddr;
  bool result = false;

  MEM_DEBUG("Packet is MEM READ\n");
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_READ_CH);

  uint8_t memId = p->data[0];
  memcpy(&memAddr, &p->data[1], 4);
  uint8_t readLen = p->data[5];
  uint8_t* startOfData = &p->data[6];

  if (memId < nrOfHandlers) {
    if (handlers[memId]->read) {
      result = handlers[memId]->read(memAddr, readLen, startOfData);
    }
  } else {
    uint8_t selectedMem = memId - nrOfHandlers;
    result = owMemHandler->read(selectedMem, memAddr, readLen, startOfData);
  }

  p->data[5] = result ? STATUS_OK : EIO;
  if (result) {
    p->size = 6 + readLen;
  } else {
    p->size = 6;
  }

  crtpSendPacket(p);
}

static void memWriteProcess(CRTPPacket* p) {
  uint32_t memAddr;
  bool result = false;

  uint8_t memId = p->data[0];
  memcpy(&memAddr, &p->data[1], 4);
  uint8_t* startOfData = &p->data[5];

  uint8_t writeLen = p->size - 5;

  MEM_DEBUG("Packet is MEM WRITE\n");
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_WRITE_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  if (memId < nrOfHandlers) {
    if (handlers[memId]->write) {
      result = handlers[memId]->write(memAddr, writeLen, startOfData);
    }
  } else {
    uint8_t selectedMem = memId - nrOfHandlers;
    result = owMemHandler->write(selectedMem, memAddr, writeLen, startOfData);
  }

  p->data[5] = result ? STATUS_OK : EIO;
  p->size = 6;

  crtpSendPacket(p);
}

/**
 * @brief The memory tester is used to verify the functionality of the memory sub system.
 * It supports "virtual" read and writes that are used by a test script to
 * check that a client (for instance the python lib) is working as expected.
 *
 * When reading data from the tester, it simply fills up a buffer with known data so that the
 * client can examine the data and verify that buffers have been correctly assembled.
 *
 * @param memAddr - the virtual address to read from
 * @param readLen - nr of bytes to read
 * @param startOfData - address to write result to
 * @return Always returns true
 */
static bool handleMemTesterRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData) {
  for (int i = 0; i < readLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t data = addr & 0xff;
    startOfData[i] = data;
  }

  return true;
}

/**
 * @brief When writing data to the tester, the tester verifies that the received data
 * contains the expected values.
 *
 * @param memAddr - the virtual address to write to
 * @param writeLen - nr of bytes to write
 * @param startOfData - pointer to the data in the packet that is provided by the client
 * @return Always returns true
 */
static bool handleMemTesterWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData) {
  if (memTesterWriteReset) {
    memTesterWriteReset = 0;
    memTesterWriteErrorCount = 0;
  }

  for (int i = 0; i < writeLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t expectedData = addr & 0xff;
    uint8_t actualData = startOfData[i];
    if (actualData != expectedData) {
      // Log first error
      if (memTesterWriteErrorCount == 0) {
        DEBUG_PRINT("Verification failed: expected: %d, actual: %d, addr: %"PRIu32"\n", expectedData, actualData, addr);
      }

      memTesterWriteErrorCount++;
      break;
    }
  }

  return true;
}

PARAM_GROUP_START(memTst)
  PARAM_ADD(PARAM_UINT8, resetW, &memTesterWriteReset)
PARAM_GROUP_STOP(memTst)

LOG_GROUP_START(memTst)
  LOG_ADD(LOG_UINT32, errCntW, &memTesterWriteErrorCount)
LOG_GROUP_STOP(memTst)
