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
 * ow.c - One-wire functions
 */
#define DEBUG_MODULE "OW"

#include  <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "ow.h"
#include "assert.h"
#include "debug_cf.h"

static xSemaphoreHandle waitForReply;
static xSemaphoreHandle lockCmdBuf;
static StaticSemaphore_t lockCmdBufBuffer;
static OwCommand owCmdBuf;
static bool owDataIsValid;

static bool owSyslinkTransfer(uint8_t type, uint8_t length);

#ifdef OW_WRITE_TEST
static uint8_t bqtestData[] =
{
  0xEB, 0x00, 0x00, 0x00, 0x00, 0xBC, 0xFE, 0x3C, 0x00, 0x0D, 0x01, 0x08,
  0x62, 0x63, 0x42, 0x51, 0x74, 0x65, 0x73, 0x74, 0x02, 0x01, 0x61, 0x70
};
#endif


void owInit()
{
  syslinkInit();
  vSemaphoreCreateBinary(waitForReply);
  lockCmdBuf = xSemaphoreCreateMutexStatic(&lockCmdBufBuffer);

  // Put reply semaphore in right state.
  xSemaphoreTake(waitForReply, portMAX_DELAY);

  owCommonInit();
}

bool owTest()
{
  uint8_t nOwMem = 0;
  uint8_t nOwIter = 0;
  OwSerialNum sn;

  if (owScan(&nOwMem))
  {
    DEBUG_PRINT("Found %d.\n", nOwMem);
  }
  else
  {
    DEBUG_PRINT("Scan [FAILED].\n");
  }

  for (nOwIter = 0; nOwIter < nOwMem; nOwIter++)
  {
    if (owGetinfo(nOwIter, &sn))
    {
      DEBUG_PRINT("Serial 0x%X %X %X %X %X %X %X %X.\n",
                  sn.type, sn.id[0], sn.id[1], sn.id[2],
                  sn.id[3], sn.id[4], sn.id[5], sn.crc);
    }
    else
    {
      DEBUG_PRINT("Mem:%d Getinfo [FAILED].\n", nOwIter);
    }
  }

#ifdef OW_READ_TEST
  {
    static uint8_t testbuf[129];

    if (owRead(0, 0, OW_MAX_SIZE, testbuf))
    {
      for (nOwIter = 0; nOwIter < OW_MAX_SIZE; nOwIter++)
      {
        consolePrintf("%X ", testbuf[nOwIter]);
        testbuf[nOwIter] = nOwIter;
      }
      consolePrintf("\n");
    }
  }
#endif
#ifdef OW_WRITE_TEST
  if (owWrite(0, 0, sizeof(bqtestData), bqtestData))
  {
    DEBUG_PRINT("Write [OK].\n");
  }
  else
  {
    DEBUG_PRINT("Write [FAIL].\n");
  }
#endif

  return owCommonTest();
}

void owSyslinkRecieve(SyslinkPacket *slp)
{
  switch (slp->type)
  {
    case SYSLINK_OW_SCAN:
    case SYSLINK_OW_GETINFO:
    case SYSLINK_OW_READ:
    case SYSLINK_OW_WRITE:
      memcpy(&owCmdBuf, slp->data, sizeof(OwCommand));
      //DEBUG_PRINT("t:%X n:%d:%X\n", slp->type, owCmdBuf.nmem, owCmdBuf.info.memId[0]);
      owDataIsValid = true;
      break;
    default:
      // Unknown reply
      owDataIsValid = false;
      break;
  }
  xSemaphoreGive(waitForReply);
}

static bool owSyslinkTransfer(uint8_t type, uint8_t length)
{
  SyslinkPacket slp;

  ASSERT(length <= SYSLINK_MTU);

  slp.type = type;
  slp.length = length;
  memcpy(slp.data, &owCmdBuf, length);

  syslinkSendPacket(&slp);
  // Wait for reply
  if (xSemaphoreTake(waitForReply, M2T(5000)) == pdTRUE)
  //if (xSemaphoreTake(waitForReply, portMAX_DELAY))
  {
    // We have now got a reply and *owCmd has been filled with data
    if (owDataIsValid)
    {
      owDataIsValid = false;
      return true;
    }
  }
  else
  {
    DEBUG_PRINT("Cmd 0x%X timeout.\n", slp.type);
  }

  return false;
}

bool owScan(uint8_t *nMem)
{
  bool status = false;

  xSemaphoreTake(lockCmdBuf, portMAX_DELAY);

  if (owSyslinkTransfer(SYSLINK_OW_SCAN, 0))
  {
    *nMem = owCmdBuf.nmem;
    status = true;
  }
  else
  {
    status = false;
  }

  xSemaphoreGive(lockCmdBuf);

  return status;
}

bool owGetinfo(uint8_t selectMem, OwSerialNum *serialNum)
{
  bool status = false;

  xSemaphoreTake(lockCmdBuf, portMAX_DELAY);
  owCmdBuf.nmem = selectMem;

  if (owSyslinkTransfer(SYSLINK_OW_GETINFO, 1))
  {
    memcpy(serialNum, owCmdBuf.info.memId, sizeof(OwSerialNum));
    if (owCmdBuf.nmem != 0xFF)
    {
      status = true;
    }
  }
  else
  {
    status = false;
  }

  xSemaphoreGive(lockCmdBuf);

  return status;
}

bool owRead(uint8_t selectMem, uint16_t address, uint8_t length, uint8_t *data)
{
  bool status = true;
  uint16_t currAddr = address;
  uint16_t endAddr = address + length;
  uint8_t bytesRead = 0;

  ASSERT(length <= OW_MAX_SIZE);

  xSemaphoreTake(lockCmdBuf, portMAX_DELAY);
  owCmdBuf.nmem = selectMem;

  while (currAddr < endAddr)
  {
    if (endAddr - currAddr < OW_READ_SIZE)
    {
      owCmdBuf.read.address = currAddr;
      if (owSyslinkTransfer(SYSLINK_OW_READ, 3 + endAddr - currAddr))
      {
        memcpy(data + bytesRead, owCmdBuf.read.data, endAddr - currAddr);
        currAddr += endAddr - currAddr;
        bytesRead += endAddr - currAddr;
      }
      else
      {
        status = false;
        break;
      }
    }
    else // Size is bigger then OW_READ_SIZE
    {
      owCmdBuf.read.address = currAddr;
      if (owSyslinkTransfer(SYSLINK_OW_READ, 3 + OW_READ_SIZE))
      {
        memcpy(data + bytesRead, owCmdBuf.read.data, OW_READ_SIZE);
        currAddr += OW_READ_SIZE;
        bytesRead += OW_READ_SIZE;
      }
      else
      {
        status = false;
        break;
      }
    }
  }

  xSemaphoreGive(lockCmdBuf);

  return status;
}

bool owWrite(uint8_t selectMem, uint16_t address, uint8_t length, const uint8_t *data)
{
  bool status = true;
  uint16_t currAddr = address;
  uint16_t endAddr = address + length;
  uint8_t bytesWritten = 0;

  ASSERT(length <= OW_MAX_SIZE);

  xSemaphoreTake(lockCmdBuf, portMAX_DELAY);
  owCmdBuf.nmem = selectMem;

  while (currAddr < endAddr)
  {
    if (endAddr - currAddr < OW_MAX_WRITE_SIZE)
    {
      owCmdBuf.write.address = currAddr;
      owCmdBuf.write.length = endAddr - currAddr;
      memcpy(owCmdBuf.write.data, data + bytesWritten, owCmdBuf.write.length);
      if (owSyslinkTransfer(SYSLINK_OW_WRITE, 5 + owCmdBuf.write.length))
      {
        currAddr += endAddr - currAddr;
        bytesWritten += endAddr - currAddr;
      }
      else
      {
        status = false;
        break;
      }
    }
    else // Size is bigger then OW_MAX_WRITE_SIZE
    {
      owCmdBuf.write.address = currAddr;
      owCmdBuf.write.length = OW_MAX_WRITE_SIZE;
      memcpy(owCmdBuf.write.data, data + bytesWritten, owCmdBuf.write.length);
      if (owSyslinkTransfer(SYSLINK_OW_WRITE, 5 + owCmdBuf.write.length))
      {
        currAddr += OW_MAX_WRITE_SIZE;
        bytesWritten += OW_MAX_WRITE_SIZE;
      }
      else
      {
        status = false;
        break;
      }
    }
  }

  xSemaphoreGive(lockCmdBuf);

  return status;
}
