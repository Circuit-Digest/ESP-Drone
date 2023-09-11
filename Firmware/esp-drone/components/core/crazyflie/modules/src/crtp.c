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
 * crtp.c - CrazyRealtimeTransferProtocol stack
 */

#include <stdbool.h>
#include <errno.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"

#include "crtp.h"
#include "info.h"
#include "cfassert.h"
#include "queuemonitor.h"
#include "static_mem.h"

#include "log.h"

#define DEBUG_MODULE "CRTP"
#include "debug_cf.h"
#include "stm32_legacy.h"


static bool isInit;

static int nopFunc(void);
static struct crtpLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
};

static struct crtpLinkOperations *link = &nopLink;

#define STATS_INTERVAL 500
static struct {
  uint32_t rxCount;
  uint32_t txCount;

  uint16_t rxRate;
  uint16_t txRate;

  uint32_t nextStatisticsTime;
  uint32_t previousStatisticsTime;
} stats;

static xQueueHandle  txQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 120
#define CRTP_RX_QUEUE_SIZE 16

static void crtpTxTask(void *param);
static void crtpRxTask(void *param);

static xQueueHandle queues[CRTP_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];
static void updateStats();

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtpTxTask, CRTP_TX_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtpRxTask, CRTP_RX_TASK_STACKSIZE);

void crtpInit(void)
{
  if(isInit)
    return;

  txQueue = xQueueCreate(CRTP_TX_QUEUE_SIZE, sizeof(CRTPPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(txQueue);

  STATIC_MEM_TASK_CREATE(crtpTxTask, crtpTxTask, CRTP_TX_TASK_NAME, NULL, CRTP_TX_TASK_PRI);
  STATIC_MEM_TASK_CREATE(crtpRxTask, crtpRxTask, CRTP_RX_TASK_NAME, NULL, CRTP_RX_TASK_PRI);

  isInit = true;
}

bool crtpTest(void)
{
  return isInit;
}

void crtpInitTaskQueue(CRTPPort portId)
{
  ASSERT(queues[portId] == NULL);

  queues[portId] = xQueueCreate(CRTP_RX_QUEUE_SIZE, sizeof(CRTPPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(queues[portId]);
}

int crtpReceivePacket(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);

  return xQueueReceive(queues[portId], p, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);

  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait)
{
  ASSERT(queues[portId]);
  ASSERT(p);

  return xQueueReceive(queues[portId], p, M2T(wait));
}

int crtpGetFreeTxQueuePackets(void)
{
  return (CRTP_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}

void crtpTxTask(void *param)
{
  CRTPPacket p;

  while (true)
  {
    if (link != &nopLink)
    {
      if (xQueueReceive(txQueue, &p, portMAX_DELAY) == pdTRUE)
      {
        // Keep testing, if the link changes to USB it will go though
        while (link->sendPacket(&p) == false)
        {
          // Relaxation time
          vTaskDelay(M2T(10));
        }
        stats.txCount++;
        updateStats();
      }
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
}

void crtpRxTask(void *param)
{
  CRTPPacket p;

  while (true)
  {
    if (link != &nopLink)
    {
      if (!link->receivePacket(&p))
      {
        if (queues[p.port])
        {
          if (xQueueSend(queues[p.port], &p, 0) == errQUEUE_FULL)
          {
            // We should never drop packet
            ASSERT(0);
          }
        }

        if (callbacks[p.port])
        {
          callbacks[p.port](&p);
        }

        stats.rxCount++;
        updateStats();
      }
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
}

void crtpRegisterPortCB(int port, CrtpCallback cb)
{
  if (port>CRTP_NBR_OF_PORTS)
    return;

  callbacks[port] = cb;
}

int crtpSendPacket(CRTPPacket *p)
{
  ASSERT(p);
  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  return xQueueSend(txQueue, p, 0);
}

int crtpSendPacketBlock(CRTPPacket *p)
{
  ASSERT(p);
  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  return xQueueSend(txQueue, p, portMAX_DELAY);
}

int crtpReset(void)
{
  xQueueReset(txQueue);
  if (link->reset) {
    link->reset();
  }

  return 0;
}

bool crtpIsConnected(void)
{
  if (link->isConnected)
    return link->isConnected();
  return true;
}

void crtpSetLink(struct crtpLinkOperations * lk)
{
  if(link)
    link->setEnable(false);

  if (lk)
    link = lk;
  else
    link = &nopLink;

  link->setEnable(true);
}

static int nopFunc(void)
{
  return ENETDOWN;
}

static void clearStats()
{
  stats.rxCount = 0;
  stats.txCount = 0;
}

static void updateStats()
{
  uint32_t now = xTaskGetTickCount();
  if (now > stats.nextStatisticsTime) {
    float interval = now - stats.previousStatisticsTime;
    stats.rxRate = (uint16_t)(1000.0f * stats.rxCount / interval);
    stats.txRate = (uint16_t)(1000.0f * stats.txCount / interval);

    clearStats();
    stats.previousStatisticsTime = now;
    stats.nextStatisticsTime = now + STATS_INTERVAL;
  }
}

LOG_GROUP_START(crtp)
LOG_ADD(LOG_UINT16, rxRate, &stats.rxRate)
LOG_ADD(LOG_UINT16, txRate, &stats.txRate)
LOG_GROUP_STOP(tdoa)
