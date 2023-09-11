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
 * log.c: Dynamic log system
 */

/* The TOC logic is mainly based on param.c
 * FIXME: See if we can factorise the TOC code */

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"
#include "crtp.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"

#include "console.h"
#include "cfassert.h"
#define DEBUG_MODULE "LOG"
#include "debug_cf.h"
#include "stm32_legacy.h"
#include "static_mem.h"

#if 0
#define LOG_DEBUG(fmt, ...) DEBUG_PRINTD("D/log " fmt, ## __VA_ARGS__)
#define LOG_ERROR(fmt, ...) DEBUG_PRINTD("E/log " fmt, ## __VA_ARGS__)
#else
#define LOG_DEBUG(...)
#define LOG_ERROR(...)
#endif


static const uint8_t typeLength[] = {
  [LOG_UINT8]  = 1,
  [LOG_UINT16] = 2,
  [LOG_UINT32] = 4,
  [LOG_INT8]   = 1,
  [LOG_INT16]  = 2,
  [LOG_INT32]  = 4,
  [LOG_FLOAT]  = 4,
  [LOG_FP16]   = 2,
};

#define TYPE_MASK (0x0f)

typedef enum {
  acqType_memory = 0,
  acqType_function = 1,
} acquisitionType_t;

// Maximum log payload length (4 bytes are used for block id and timestamp)
#define LOG_MAX_LEN 26

/* Log packet parameters storage */
#define LOG_MAX_OPS 128
#define LOG_MAX_BLOCKS 16
struct log_ops {
  struct log_ops * next;
  uint8_t storageType : 4;
  uint8_t logType     : 4;
  void * variable;
  acquisitionType_t acquisitionType;
};

struct log_block {
  int id;
  xTimerHandle timer;
  StaticTimer_t timerBuffer;
  struct log_ops * ops;
};

NO_DMA_CCM_SAFE_ZERO_INIT static struct log_ops logOps[LOG_MAX_OPS];
NO_DMA_CCM_SAFE_ZERO_INIT static struct log_block logBlocks[LOG_MAX_BLOCKS];
static xSemaphoreHandle logLock;
static StaticSemaphore_t logLockBuffer;

struct ops_setting {
    uint8_t logType;
    uint8_t id;
} __attribute__((packed));

struct ops_setting_v2 {
    uint8_t logType;
    uint16_t id;
} __attribute__((packed));


#define TOC_CH      0
#define CONTROL_CH  1
#define LOG_CH      2

#define CMD_GET_ITEM    0 // original version: up to 255 entries
#define CMD_GET_INFO    1 // original version: up to 255 entries
#define CMD_GET_ITEM_V2 2 // version 2: up to 16k entries
#define CMD_GET_INFO_V2 3 // version 2: up to 16k entries

#define CONTROL_CREATE_BLOCK    0
#define CONTROL_APPEND_BLOCK    1
#define CONTROL_DELETE_BLOCK    2
#define CONTROL_START_BLOCK     3
#define CONTROL_STOP_BLOCK      4
#define CONTROL_RESET           5
#define CONTROL_CREATE_BLOCK_V2 6
#define CONTROL_APPEND_BLOCK_V2 7

#define BLOCK_ID_FREE -1

//Private functions
static void logTask(void * prm);
static void logTOCProcess(int command);
static void logControlProcess(void);

void logRunBlock(void * arg);
void logBlockTimed(xTimerHandle timer);

//These are set by the Linker
extern struct log_s _log_start;
extern struct log_s _log_end;

//Pointer to the logeters list and length of it
static struct log_s * logs;
static int logsLen;
static uint32_t logsCrc;
static uint16_t logsCount = 0;

static CRTPPacket p;

static bool isInit = false;

/* Log management functions */
static int logAppendBlock(int id, struct ops_setting * settings, int len);
static int logAppendBlockV2(int id, struct ops_setting_v2 * settings, int len);
static int logCreateBlock(unsigned char id, struct ops_setting * settings, int len);
static int logCreateBlockV2(unsigned char id, struct ops_setting_v2 * settings, int len);
static int logDeleteBlock(int id);
static int logStartBlock(int id, unsigned int period);
static int logStopBlock(int id);
static void logReset();
static acquisitionType_t acquisitionTypeFromLogType(uint8_t logType);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(logTask, LOG_TASK_STACKSIZE);

void logInit(void)
{
  int i;
  const char* group = NULL;
  int groupLength = 0;

  if(isInit)
    return;

  logs = &_log_start;
  logsLen = &_log_end - &_log_start;

  // Calculate a hash of the toc by chaining description of each elements
  // Using the CRTP packet as temporary buffer
  logsCrc = 0;
  for (int i=0; i<logsLen; i++)
  {
    int len = 5;
    memcpy(&p.data[0], &logsCrc, 4);
    p.data[4] = logs[i].type;
    if (logs[i].type & LOG_GROUP) {
      if (logs[i].type & LOG_START) {
        group = logs[i].name;
        groupLength = strlen(group);
      }
    } else {
      // CMD_GET_ITEM_V2 result's size is: 3 + strlen(logs[i].name) + groupLength + 2
      if (strlen(logs[i].name) + groupLength + 2 > 26) {
        LOG_ERROR("'%s.%s' too long\n", group, logs[i].name);
        ASSERT_FAILED();
      }
    }
    if (logs[i].name) {
      memcpy(&p.data[5], logs[i].name, strlen(logs[i].name));
      len += strlen(logs[i].name);
    }
    logsCrc = crcSlow(p.data, len);
  }

  // Big lock that protects the log datastructures
  logLock = xSemaphoreCreateMutexStatic(&logLockBuffer);

  for (i=0; i<logsLen; i++)
  {
    if(!(logs[i].type & LOG_GROUP))
      logsCount++;
  }

  //Manually free all log blocks
  for(i=0; i<LOG_MAX_BLOCKS; i++)
    logBlocks[i].id = BLOCK_ID_FREE;

  //Init data structures and set the log subsystem in a known state
  logReset();

  //Start the log task
  STATIC_MEM_TASK_CREATE(logTask, logTask, LOG_TASK_NAME, NULL, LOG_TASK_PRI);

  isInit = true;
}

bool logTest(void)
{
  return isInit;
}

void logTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_LOG);

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_LOG, &p);

		xSemaphoreTake(logLock, portMAX_DELAY);
		if (p.channel==TOC_CH)
		  logTOCProcess(p.data[0]);
		if (p.channel==CONTROL_CH)
		  logControlProcess();
		xSemaphoreGive(logLock);
	}
}

void logTOCProcess(int command)
{
  int ptr = 0;
  char * group = "plop";
  uint16_t n=0;
  uint16_t logId=0;

  switch (command)
  {
  case CMD_GET_INFO: //Get info packet about the log implementation
    DEBUG_PRINT("Client uses old logging API!\n");
    LOG_DEBUG("Packet is TOC_GET_INFO\n");
    ptr = 0;
    group = "";
    p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
    p.size=8;
    p.data[0]=CMD_GET_INFO;
    if (logsCount < 255) {
      p.data[1]=logsCount;
    } else {
      p.data[1]=255;
    }
    memcpy(&p.data[2], &logsCrc, 4);
    p.data[6]=LOG_MAX_BLOCKS;
    p.data[7]=LOG_MAX_OPS;
    crtpSendPacket(&p);
    break;
  case CMD_GET_ITEM:  //Get log variable
    LOG_DEBUG("Packet is TOC_GET_ITEM Id: %d\n", p.data[1]);
    for (ptr=0; ptr<logsLen; ptr++) //Ptr points a group
    {
      if (logs[ptr].type & LOG_GROUP)
      {
        if (logs[ptr].type & LOG_START)
          group = logs[ptr].name;
        else
          group = "";
      }
      else                          //Ptr points a variable
      {
        if (n==p.data[1])
          break;
        n++;
      }
    }

    if (ptr<logsLen)
    {
      LOG_DEBUG("    Item is \"%s\":\"%s\"\n", group, logs[ptr].name);
      p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.data[1]=n;
      p.data[2]=logs[ptr].type & TYPE_MASK;
      p.size=3+2+strlen(group)+strlen(logs[ptr].name);
      ASSERT(p.size <= CRTP_MAX_DATA_SIZE); // Too long! The name of the group or the parameter may be too long.
      memcpy(p.data+3, group, strlen(group)+1);
      memcpy(p.data+3+strlen(group)+1, logs[ptr].name, strlen(logs[ptr].name)+1);
      crtpSendPacket(&p);
    } else {
      LOG_DEBUG("    Index out of range!");
      p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
      p.data[0]=CMD_GET_ITEM;
      p.size=1;
      crtpSendPacket(&p);
    }
    break;
  case CMD_GET_INFO_V2: //Get info packet about the log implementation
    LOG_DEBUG("Packet is TOC_GET_INFO\n");
    ptr = 0;
    group = "";
    p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
    p.size=9;
    p.data[0]=CMD_GET_INFO_V2;
    memcpy(&p.data[1], &logsCount, 2);
    memcpy(&p.data[3], &logsCrc, 4);
    p.data[7]=LOG_MAX_BLOCKS;
    p.data[8]=LOG_MAX_OPS;
    crtpSendPacket(&p);
    break;
  case CMD_GET_ITEM_V2:  //Get log variable
    memcpy(&logId, &p.data[1], 2);
    LOG_DEBUG("Packet is TOC_GET_ITEM Id: %d\n", logId);
    for (ptr=0; ptr<logsLen; ptr++) //Ptr points a group
    {
      if (logs[ptr].type & LOG_GROUP)
      {
        if (logs[ptr].type & LOG_START)
          group = logs[ptr].name;
        else
          group = "";
      }
      else                          //Ptr points a variable
      {
        if (n==logId)
          break;
        n++;
      }
    }

    if (ptr<logsLen)
    {
      LOG_DEBUG("    Item is \"%s\":\"%s\"\n", group, logs[ptr].name);
      p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
      p.data[0]=CMD_GET_ITEM_V2;
      memcpy(&p.data[1], &logId, 2);
      p.data[3]=logs[ptr].type & TYPE_MASK;
      p.size=4+2+strlen(group)+strlen(logs[ptr].name);
      ASSERT(p.size <= CRTP_MAX_DATA_SIZE); // Too long! The name of the group or the parameter may be too long.
      memcpy(p.data+4, group, strlen(group)+1);
      memcpy(p.data+4+strlen(group)+1, logs[ptr].name, strlen(logs[ptr].name)+1);
      crtpSendPacket(&p);
    } else {
      LOG_DEBUG("    Index out of range!");
      p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
      p.data[0]=CMD_GET_ITEM_V2;
      p.size=1;
      crtpSendPacket(&p);
    }
    break;
  }
}

void logControlProcess()
{
  int ret = ENOEXEC;

  switch(p.data[0])
  {
    case CONTROL_CREATE_BLOCK:
      ret = logCreateBlock( p.data[1],
                            (struct ops_setting*)&p.data[2],
                            (p.size-2)/sizeof(struct ops_setting) );
      break;
    case CONTROL_APPEND_BLOCK:
      ret = logAppendBlock( p.data[1],
                            (struct ops_setting*)&p.data[2],
                            (p.size-2)/sizeof(struct ops_setting) );
      break;
    case CONTROL_DELETE_BLOCK:
      ret = logDeleteBlock( p.data[1] );
      break;
    case CONTROL_START_BLOCK:
      ret = logStartBlock( p.data[1], p.data[2]*10);
      break;
    case CONTROL_STOP_BLOCK:
      ret = logStopBlock( p.data[1] );
      break;
    case CONTROL_RESET:
      logReset();
      ret = 0;
      break;
    case CONTROL_CREATE_BLOCK_V2:
      ret = logCreateBlockV2( p.data[1],
                            (struct ops_setting_v2*)&p.data[2],
                            (p.size-2)/sizeof(struct ops_setting_v2) );
      break;
    case CONTROL_APPEND_BLOCK_V2:
      ret = logAppendBlockV2( p.data[1],
                            (struct ops_setting_v2*)&p.data[2],
                            (p.size-2)/sizeof(struct ops_setting_v2) );
      break;
  }

  //Commands answer
  p.data[2] = ret;
  p.size = 3;
  crtpSendPacket(&p);
}

static int logCreateBlock(unsigned char id, struct ops_setting * settings, int len)
{
  int i;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (id == logBlocks[i].id) return EEXIST;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == BLOCK_ID_FREE) break;

  if (i == LOG_MAX_BLOCKS)
    return ENOMEM;

  logBlocks[i].id = id;
  logBlocks[i].timer = xTimerCreateStatic("logTimer", M2T(1000), pdTRUE,
    &logBlocks[i], logBlockTimed, &logBlocks[i].timerBuffer);
  logBlocks[i].ops = NULL;

  if (logBlocks[i].timer == NULL)
  {
	logBlocks[i].id = BLOCK_ID_FREE;
	return ENOMEM;
  }

  LOG_DEBUG("Added block ID %d\n", id);

  return logAppendBlock(id, settings, len);
}

static int logCreateBlockV2(unsigned char id, struct ops_setting_v2 * settings, int len)
{
  int i;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (id == logBlocks[i].id) return EEXIST;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == BLOCK_ID_FREE) break;

  if (i == LOG_MAX_BLOCKS)
    return ENOMEM;

  logBlocks[i].id = id;
  logBlocks[i].timer = xTimerCreateStatic("logTimer", M2T(1000), pdTRUE,
    &logBlocks[i], logBlockTimed, &logBlocks[i].timerBuffer);
  logBlocks[i].ops = NULL;

  if (logBlocks[i].timer == NULL)
  {
  logBlocks[i].id = BLOCK_ID_FREE;
  return ENOMEM;
  }

  LOG_DEBUG("Added block ID %d\n", id);

  return logAppendBlockV2(id, settings, len);
}

static int blockCalcLength(struct log_block * block);
static struct log_ops * opsMalloc();
static void opsFree(struct log_ops * ops);
static void blockAppendOps(struct log_block * block, struct log_ops * ops);
static int variableGetIndex(int id);

static int logAppendBlock(int id, struct ops_setting * settings, int len)
{
  int i;
  struct log_block * block;

  LOG_DEBUG("Appending %d variable to block %d\n", len, id);

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG_ERROR("Trying to append block id %d that doesn't exist.", id);
    return ENOENT;
  }

  block = &logBlocks[i];

  for (i=0; i<len; i++)
  {
    int currentLength = blockCalcLength(block);
    struct log_ops * ops;
    int varId;

    if ((currentLength + typeLength[settings[i].logType & TYPE_MASK])>LOG_MAX_LEN) {
      LOG_ERROR("Trying to append a full block. Block id %d.\n", id);
      return E2BIG;
    }

    ops = opsMalloc();

    if(!ops) {
      LOG_ERROR("No more ops memory free!\n");
      return ENOMEM;
    }

    if (settings[i].id != 255)  //TOC variable
    {
      varId = variableGetIndex(settings[i].id);

      if (varId<0) {
        LOG_ERROR("Trying to add variable Id %d that does not exists.", settings[i].id);
        return ENOENT;
      }

      ops->variable    = logs[varId].address;
      ops->storageType = logs[varId].type & TYPE_MASK;
      ops->logType     = settings[i].logType & TYPE_MASK;
      ops->acquisitionType = acquisitionTypeFromLogType(logs[varId].type);

      LOG_DEBUG("Appended variable %d to block %d\n", settings[i].id, id);
    } else {                     //Memory variable
      //TODO: Check that the address is in ram
      ops->variable    = (void*)(&settings[i]+1);
      ops->storageType = (settings[i].logType>>4) & TYPE_MASK;
      ops->logType     = settings[i].logType & TYPE_MASK;
      ops->acquisitionType = acqType_memory;
      i += 2;

      LOG_DEBUG("Appended var addr 0x%x to block %d\n", (int)ops->variable, id);
    }
    blockAppendOps(block, ops);

    LOG_DEBUG("   Now lenght %d\n", blockCalcLength(block));
  }

  return 0;
}

static int logAppendBlockV2(int id, struct ops_setting_v2 * settings, int len)
{
  int i;
  struct log_block * block;

  LOG_DEBUG("Appending %d variable to block %d\n", len, id);

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG_ERROR("Trying to append block id %d that doesn't exist.", id);
    return ENOENT;
  }

  block = &logBlocks[i];

  for (i=0; i<len; i++)
  {
    int currentLength = blockCalcLength(block);
    struct log_ops * ops;
    int varId;

    if ((currentLength + typeLength[settings[i].logType & TYPE_MASK])>LOG_MAX_LEN) {
      LOG_ERROR("Trying to append a full block. Block id %d.\n", id);
      return E2BIG;
    }

    ops = opsMalloc();

    if(!ops) {
      LOG_ERROR("No more ops memory free!\n");
      return ENOMEM;
    }

    if (settings[i].id != 0xFFFFul)  //TOC variable
    {
      varId = variableGetIndex(settings[i].id);

      if (varId<0) {
        LOG_ERROR("Trying to add variable Id %d that does not exists.", settings[i].id);
        return ENOENT;
      }

      ops->variable    = logs[varId].address;
      ops->storageType = logs[varId].type & TYPE_MASK;
      ops->logType     = settings[i].logType & TYPE_MASK;
      ops->acquisitionType = acquisitionTypeFromLogType(logs[varId].type);

      LOG_DEBUG("Appended variable %d to block %d\n", settings[i].id, id);
    } else {                     //Memory variable
      //TODO: Check that the address is in ram
      ops->variable    = (void*)(&settings[i]+1);
      ops->storageType = (settings[i].logType>>4) & TYPE_MASK;
      ops->logType     = settings[i].logType & TYPE_MASK;
      ops->acquisitionType = acqType_memory;
      i += 2;

      LOG_DEBUG("Appended var addr 0x%x to block %d\n", (int)ops->variable, id);
    }
    blockAppendOps(block, ops);

    LOG_DEBUG("   Now lenght %d\n", blockCalcLength(block));
  }

  return 0;
}

static int logDeleteBlock(int id)
{
  int i;
  struct log_ops * ops;
  struct log_ops * opsNext;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG_ERROR("trying to delete block id %d that doesn't exist.", id);
    return ENOENT;
  }

  ops = logBlocks[i].ops;
  while (ops)
  {
    opsNext = ops->next;
    opsFree(ops);
    ops = opsNext;
  }

  if (logBlocks[i].timer != 0) {
    xTimerStop(logBlocks[i].timer, portMAX_DELAY);
    xTimerDelete(logBlocks[i].timer, portMAX_DELAY);
    logBlocks[i].timer = 0;
  }

  logBlocks[i].id = BLOCK_ID_FREE;
  return 0;
}

static int logStartBlock(int id, unsigned int period)
{
  int i;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG_ERROR("Trying to start block id %d that doesn't exist.", id);
    return ENOENT;
  }

  LOG_DEBUG("Starting block %d with period %dms\n", id, period);

  if (period>0)
  {
    xTimerChangePeriod(logBlocks[i].timer, M2T(period), 100);
    xTimerStart(logBlocks[i].timer, 100);
  } else {
    // single-shoot run
    workerSchedule(logRunBlock, &logBlocks[i]);
  }

  return 0;
}

static int logStopBlock(int id)
{
  int i;

  for (i=0; i<LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG_ERROR("Trying to stop block id %d that doesn't exist.\n", id);
    return ENOENT;
  }

  xTimerStop(logBlocks[i].timer, portMAX_DELAY);

  return 0;
}

/* This function is called by the timer subsystem */
void logBlockTimed(xTimerHandle timer)
{
  workerSchedule(logRunBlock, pvTimerGetTimerID(timer));
}

/* Appends data to a packet if space is available; returns false on failure. */
static bool appendToPacket(CRTPPacket * pk, const void * data, size_t n) {
  if (pk->size <= CRTP_MAX_DATA_SIZE - n)
  {
    memcpy(&pk->data[pk->size], data, n);
    pk->size += n;
    return true;
  }
  else return false;
}

/* This function is usually called by the worker subsystem */
void logRunBlock(void * arg)
{
  struct log_block *blk = arg;
  struct log_ops *ops = blk->ops;
  static CRTPPacket pk;
  unsigned int timestamp;

  xSemaphoreTake(logLock, portMAX_DELAY);

  timestamp = ((long long)xTaskGetTickCount())/portTICK_RATE_MS;

  pk.header = CRTP_HEADER(CRTP_PORT_LOG, LOG_CH);
  pk.size = 4;
  pk.data[0] = blk->id;
  pk.data[1] = timestamp&0x0ff;
  pk.data[2] = (timestamp>>8)&0x0ff;
  pk.data[3] = (timestamp>>16)&0x0ff;

  while (ops)
  {
    int valuei = 0;
    float valuef = 0;

    // FPU instructions must run on aligned data.
    // We first copy the data to an (aligned) local variable, before assigning it
    switch(ops->storageType)
    {
      case LOG_UINT8:
      {
        uint8_t v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->acquireUInt8(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(v));
        }
        valuei = v;
        break;
      }
      case LOG_INT8:
      {
        int8_t v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->acquireInt8(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(v));
        }
        valuei = v;
        break;
      }
      case LOG_UINT16:
      {
        uint16_t v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->acquireUInt16(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(v));
        }
        valuei = v;
        break;
      }
      case LOG_INT16:
      {
        int16_t v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->acquireInt16(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(v));
        }
        valuei = v;
        break;
      }
      case LOG_UINT32:
      {
        uint32_t v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->acquireUInt32(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(v));
        }
        valuei = v;
        break;
      }
      case LOG_INT32:
      {
        int32_t v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->acquireInt32(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(v));
        }
        valuei = v;
        break;
      }
      case LOG_FLOAT:
      {
        float v;
        if (ops->acquisitionType == acqType_function) {
          logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
          v = logByFunction->aquireFloat(timestamp, logByFunction->data);
        } else {
          memcpy(&v, ops->variable, sizeof(valuef));
        }
        valuei = v;
        valuef = v;
        break;
      }
    }

    if (ops->logType == LOG_FLOAT || ops->logType == LOG_FP16)
    {
      if (ops->storageType != LOG_FLOAT)
      {
        valuef = valuei;
      }

      // Try to append the next item to the packet.  If we run out of space,
      // drop this and subsequent items.
      if (ops->logType == LOG_FLOAT)
      {
        if (!appendToPacket(&pk, &valuef, 4)) break;
      }
      else
      {
        valuei = single2half(valuef);
        if (!appendToPacket(&pk, &valuei, 2)) break;
      }
    }
    else  //logType is an integer
    {
      if (!appendToPacket(&pk, &valuei, typeLength[ops->logType])) break;
    }

    ops = ops->next;
  }

  xSemaphoreGive(logLock);

  // Check if the connection is still up, oherwise disable
  // all the logging and flush all the CRTP queues.
  if (!crtpIsConnected())
  {
    logReset();
    crtpReset();
  }
  else
  {
    crtpSendPacket(&pk);
  }
}

static int variableGetIndex(int id)
{
  int i;
  int n=0;

  for (i=0; i<logsLen; i++)
  {
    if(!(logs[i].type & LOG_GROUP))
    {
      if(n==id)
        break;
      n++;
    }
  }

  if (i>=logsLen)
    return -1;

  return i;
}

static struct log_ops * opsMalloc()
{
  int i;

  for (i=0;i<LOG_MAX_OPS; i++)
      if (logOps[i].variable == NULL) break;

  if (i >= LOG_MAX_OPS)
      return NULL;

  return &logOps[i];
}

static void opsFree(struct log_ops * ops)
{
  ops->variable = NULL;
}

static int blockCalcLength(struct log_block * block)
{
  struct log_ops * ops;
  int len = 0;

  for (ops = block->ops; ops; ops = ops->next)
    len += typeLength[ops->logType];

  return len;
}

void blockAppendOps(struct log_block * block, struct log_ops * ops)
{
  struct log_ops * o;

  ops->next = NULL;

  if (block->ops == NULL)
    block->ops = ops;
  else
  {
    for (o = block->ops; o->next; o = o->next);

    o->next = ops;
  }
}

static void logReset(void)
{
  int i;

  if (isInit)
  {
    //Stop and delete all started log blocks
    for(i=0; i<LOG_MAX_BLOCKS; i++)
      if (logBlocks[i].id != -1)
      {
        logStopBlock(logBlocks[i].id);
        logDeleteBlock(logBlocks[i].id);
      }
  }

  //Force free all the log block objects
  for(i=0; i<LOG_MAX_BLOCKS; i++)
    logBlocks[i].id = BLOCK_ID_FREE;

  //Force free the log ops
  for (i=0; i<LOG_MAX_OPS; i++)
    logOps[i].variable = NULL;
}

/* Public API to access log TOC from within the copter */
static logVarId_t invalidVarId = 0xffffu;

logVarId_t logGetVarId(char* group, char* name)
{
  int i;
  logVarId_t varId = invalidVarId;
  char * currgroup = "";

  for(i=0; i<logsLen; i++)
  {
    if (logs[i].type & LOG_GROUP) {
      if (logs[i].type & LOG_START)
        currgroup = logs[i].name;
    } if ((!strcmp(group, currgroup)) && (!strcmp(name, logs[i].name))) {
      varId = (logVarId_t)i;
      return varId;
    }
  }

  return invalidVarId;
}

int logGetType(logVarId_t varid)
{
  return logs[varid].type;
}

void logGetGroupAndName(logVarId_t varid, char** group, char** name)
{
  char * currgroup = "";
  *group = 0;
  *name = 0;

  for(int i=0; i<logsLen; i++) {
    if (logs[i].type & LOG_GROUP) {
      if (logs[i].type & LOG_START) {
        currgroup = logs[i].name;
      }
    }

    if (i == varid) {
      *group = currgroup;
      *name = logs[i].name;
      break;
    }
  }
}

void* logGetAddress(logVarId_t varid)
{
  return logs[varid].address;
}

uint8_t logVarSize(int type)
{
  return typeLength[type];
}

int logGetInt(logVarId_t varid)
{
  int valuei = 0;

  ASSERT(LOG_VARID_IS_VALID(varid));

  switch(logs[varid].type)
  {
    case LOG_UINT8:
      valuei = *(uint8_t *)logs[varid].address;
      break;
    case LOG_INT8:
      valuei = *(int8_t *)logs[varid].address;
      break;
    case LOG_UINT16:
      valuei = *(uint16_t *)logs[varid].address;
      break;
    case LOG_INT16:
      valuei = *(int16_t *)logs[varid].address;
      break;
    case LOG_UINT32:
      valuei = *(uint32_t *)logs[varid].address;
      break;
    case LOG_INT32:
      valuei = *(int32_t *)logs[varid].address;
      break;
    case LOG_FLOAT:
      valuei = *(float *)logs[varid].address;
      break;
  }

  return valuei;
}

float logGetFloat(logVarId_t varid)
{
  ASSERT(LOG_VARID_IS_VALID(varid));

  if (logs[varid].type == LOG_FLOAT)
    return *(float *)logs[varid].address;

  return logGetInt(varid);
}

unsigned int logGetUint(logVarId_t varid)
{
  return (unsigned int)logGetInt(varid);
}

static acquisitionType_t acquisitionTypeFromLogType(uint8_t logType) {
  if (logType & LOG_BY_FUNCTION) {
    return acqType_function;
  }

  return acqType_memory;
}
