/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * syslink.c: nRF24L01 implementation of the CRTP link
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "usblink.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "static_mem.h"

#include "usb.h"

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static uint8_t sendBuffer[64];

static int usblinkSendPacket(CRTPPacket *p);
static int usblinkSetEnable(bool enable);
static int usblinkReceiveCRTPPacket(CRTPPacket *p);

STATIC_MEM_TASK_ALLOC(usblinkTask, USBLINK_TASK_STACKSIZE);

static struct crtpLinkOperations usblinkOp =
{
  .setEnable         = usblinkSetEnable,
  .sendPacket        = usblinkSendPacket,
  .receivePacket     = usblinkReceiveCRTPPacket,
};

/* Radio task handles the CRTP packet transfers as well as the radio link
 * specific communications (eg. Scann and ID ports, communication error handling
 * and so much other cool things that I don't have time for it ...)
 */
static USBPacket usbIn;
static CRTPPacket p;
static void usblinkTask(void *param)
{
  while(1)
  {
    // Fetch a USB packet off the queue
    usbGetDataBlocking(&usbIn);
    p.size = usbIn.size - 1;
    memcpy(&p.raw, usbIn.data, usbIn.size);
    // This queuing will copy a CRTP packet size from usbIn
    ASSERT(xQueueSend(crtpPacketDelivery, &p, 0) == pdTRUE);
  }

}

static int usblinkReceiveCRTPPacket(CRTPPacket *p)
{
  if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE)
  {
    ledseqRun(&seq_linkUp);
    return 0;
  }

  return -1;
}

static int usblinkSendPacket(CRTPPacket *p)
{
  int dataSize;

  ASSERT(p->size < SYSLINK_MTU);

  sendBuffer[0] = p->header;

  if (p->size <= CRTP_MAX_DATA_SIZE)
  {
    memcpy(&sendBuffer[1], p->data, p->size);
  }
  dataSize = p->size + 1;


  ledseqRun(&seq_linkDown);

  return usbSendData(dataSize, sendBuffer);
}

static int usblinkSetEnable(bool enable)
{
  return 0;
}

/*
 * Public functions
 */

void usblinkInit()
{
  if(isInit)
    return;

  // Initialize the USB peripheral
  usbInit();

  crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
  DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

  STATIC_MEM_TASK_CREATE(usblinkTask, usblinkTask, USBLINK_TASK_NAME, NULL, USBLINK_TASK_PRI);

  isInit = true;
}

bool usblinkTest()
{
  return isInit;
}

struct crtpLinkOperations * usblinkGetLink()
{
  return &usblinkOp;
}
