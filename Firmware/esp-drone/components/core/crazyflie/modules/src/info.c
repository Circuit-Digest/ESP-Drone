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
 * info.c - Receive information requests and send them back to the client
 */

#include <string.h>
#include <math.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "info.h"
#include "version.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#include "static_mem.h"

//CPUID access
static const unsigned int * CpuId = (unsigned int*)0x1FFFF7E8;

typedef enum {
  infoCopterNr = 0x00,
  infoBatteryNr = 0x01,
  infoWarningNr = 0x03
} InfoNbr;

typedef enum {
  infoName = 0x00,
  infoVersion = 0x01,
  infoCpuId = 0x02
} infoId;

typedef enum {
  batteryVoltage = 0x00,
  batteryMin = 0x01,
  batteryMax = 0x02
} batteryId;

typedef enum {
  warningBattery = 0x00
} warningId;

STATIC_MEM_TASK_ALLOC(infoTask, INFO_TASK_STACKSIZE);

void infoTask(void *param);

void infoInit()
{
  STATIC_MEM_TASK_CREATE(infoTask, infoTask, INFO_TASK_NAME, NULL, INFO_TASK_PRI);
  crtpInitTaskQueue(crtpInfo);
}

void infoTask(void *param)
{
  CRTPPacket p;
  int i;
  static int ctr=0;

  while (TRUE)
  {
    if (crtpReceivePacketWait(crtpInfo, &p, 1000) == pdTRUE)
    {
      InfoNbr infoNbr = CRTP_GET_NBR(p.port);

      switch (infoNbr)
      {
        case infoCopterNr:
          if (p.data[0] == infoName)
          {
            p.data[1] = 0x90;
            p.data[2] = 0x00;   //Version 0.9.0 (Crazyflie)
            strcpy((char*)&p.data[3], "CrazyFlie");

            p.size = 3+strlen("CrazyFlie");
            crtpSendPacket(&p);
          } else if (p.data[0] == infoVersion) {
            i=1;

            strncpy((char*)&p.data[i], V_SLOCAL_REVISION, 31-i);
            i += strlen(V_SLOCAL_REVISION);

            if (i<31) p.data[i++] = ',';

            strncpy((char*)&p.data[i], V_SREVISION, 31-i);
            i += strlen(V_SREVISION);

            if (i<31) p.data[i++] = ',';

            strncpy((char*)&p.data[i], V_STAG, 31-i);
            i += strlen(V_STAG);

            if (i<31) p.data[i++] = ',';
            if (i<31) p.data[i++] = V_MODIFIED?'M':'C';

            p.size = (i<31)?i:31;
            crtpSendPacket(&p);
          } else if (p.data[0] == infoCpuId) {
            memcpy((char*)&p.data[1], (char*)CpuId, 12);

            p.size = 13;
            crtpSendPacket(&p);
          }

          break;
        case infoBatteryNr:
          if (p.data[0] == batteryVoltage)
          {
            float value = pmGetBatteryVoltage();

            memcpy(&p.data[1], (char*)&value, 4);

            p.size = 5;
            crtpSendPacket(&p);
          } else if (p.data[0] == batteryMax) {
            float value = pmGetBatteryVoltageMax();

            memcpy(&p.data[1], (char*)&value, 4);

            p.size = 5;
            crtpSendPacket(&p);
          } else if (p.data[0] == batteryMin) {
            float value = pmGetBatteryVoltageMin();

            memcpy(&p.data[1], (char*)&value, 4);

            p.size = 5;
            crtpSendPacket(&p);
          }
          break;
        default:
          break;
      }
    }

    // Send a warning message if the battery voltage drops under 3.3V
    // This is sent every 5 info transaction or every 5 seconds
    if (ctr++>5) {
      ctr=0;

      if (pmGetBatteryVoltageMin() < INFO_BAT_WARNING)
      {
        float value = pmGetBatteryVoltage();

        p.port = CRTP_PORT(0,8,3);
        p.data[0] = 0;
        memcpy(&p.data[1], (char*)&value, 4);

        p.size = 5;
        crtpSendPacket(&p);
      }
    }

  }
}
