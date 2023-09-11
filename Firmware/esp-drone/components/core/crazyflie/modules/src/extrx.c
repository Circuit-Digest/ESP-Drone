/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
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
 * extrx.c - Module to handle external receiver inputs
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#include "stm32_legacy.h"
#include "config.h"
#include "system.h"
#include "nvicconf.h"
#include "commander.h"
#include "uart1.h"
#include "cppm.h"

#define DEBUG_MODULE  "EXTRX"
#include "debug_cf.h"
#include "log.h"
#include "static_mem.h"

//#define ENABLE_CPPM
#define ENABLE_EXTRX_LOG


#define EXTRX_NR_CHANNELS  8

#define EXTRX_CH_TRUST     2
#define EXTRX_CH_ROLL      0
#define EXTRX_CH_PITCH     1
#define EXTRX_CH_YAW       3

#define EXTRX_SIGN_ROLL    (-1)
#define EXTRX_SIGN_PITCH   (-1)
#define EXTRX_SIGN_YAW     (-1)

#define EXTRX_SCALE_ROLL   (40.0f)
#define EXTRX_SCALE_PITCH  (40.0f)
#define EXTRX_SCALE_YAW    (400.0f)

static setpoint_t extrxSetpoint;
static uint16_t ch[EXTRX_NR_CHANNELS];

static void extRxTask(void *param);
static void extRxDecodeCppm(void);
static void extRxDecodeChannels(void);

STATIC_MEM_TASK_ALLOC(extRxTask, EXTRX_TASK_STACKSIZE);

void extRxInit(void)
{

  extrxSetpoint.mode.roll = modeAbs;
  extrxSetpoint.mode.pitch = modeAbs;
  extrxSetpoint.mode.yaw = modeVelocity;

#ifdef ENABLE_CPPM
  cppmInit();
#endif

#ifdef ENABLE_SPEKTRUM
  uart1Init();
#endif

  STATIC_MEM_TASK_CREATE(extRxTask, extRxTask, EXTRX_TASK_NAME, NULL, EXTRX_TASK_PRI);
}

static void extRxTask(void *param)
{

  //Wait for the system to be fully started
  systemWaitStart();

  while (true)
  {
    extRxDecodeCppm();
  }
}

static void extRxDecodeChannels(void)
{
  extrxSetpoint.thrust = cppmConvert2uint16(ch[EXTRX_CH_TRUST]);
  extrxSetpoint.attitude.roll = EXTRX_SIGN_ROLL * cppmConvert2Float(ch[EXTRX_CH_ROLL], -EXTRX_SCALE_ROLL, EXTRX_SCALE_ROLL);
  extrxSetpoint.attitude.pitch = EXTRX_SIGN_PITCH * cppmConvert2Float(ch[EXTRX_CH_PITCH], -EXTRX_SCALE_PITCH, EXTRX_SCALE_PITCH);
  extrxSetpoint.attitude.yaw = EXTRX_SIGN_YAW * cppmConvert2Float(ch[EXTRX_CH_YAW], -EXTRX_SCALE_YAW, EXTRX_SCALE_YAW);
  commanderSetSetpoint(&extrxSetpoint, COMMANDER_PRIORITY_EXTRX);
}

static void extRxDecodeCppm(void)
{
  uint16_t ppm;
  static uint8_t currChannel = 0;

  if (cppmGetTimestamp(&ppm) == pdTRUE)
  {
    if (cppmIsAvailible() && ppm < 2100)
    {
      if (currChannel < EXTRX_NR_CHANNELS)
      {
        ch[currChannel] = ppm;
      }
      currChannel++;
    }
    else
    {
      extRxDecodeChannels();
      currChannel = 0;
    }
  }
}

#if 0
static void extRxDecodeSpektrum(void)
{
  while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE)
  { // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
    for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++)
    {
      SerialRead(SPEK_SERIAL_PORT);
    }  //Toss one full frame of bytes.
  }
  if (spekFrameFlags == 0x01)
  { //The interrupt handler saw at least one valid frame start since we were last here.
    if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE)
    {  //A complete frame? If not, we'll catch it next time we are called.
      SerialRead(SPEK_SERIAL_PORT);
      SerialRead(SPEK_SERIAL_PORT);        //Eat the header bytes
      for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2)
      {
        uint8_t bh = SerialRead(SPEK_SERIAL_PORT);
        uint8_t bl = SerialRead(SPEK_SERIAL_PORT);
        uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
      if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
    }
    spekFrameFlags = 0x00;
    spekFrameData = 0x01;
#if defined(FAILSAFE)
    if(failsafeCnt > 20) failsafeCnt -= 20;
    else failsafeCnt = 0;   // Valid frame, clear FailSafe counter
#endif
  }
  else
  { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
    uint32_t spekInterval = (timer0_overflow_count << 8)
        * (64 / clockCyclesPerMicrosecond()) - spekTimeLast;
    if (spekInterval > 2500)
    {
      spekFrameFlags = 0;
    }  //If it has been a while, make the interrupt handler start over.
  }
}
#endif

/* Loggable variables */
#ifdef ENABLE_EXTRX_LOG
LOG_GROUP_START(extrx)
LOG_ADD(LOG_UINT16, ch0, &ch[0])
LOG_ADD(LOG_UINT16, ch1, &ch[1])
LOG_ADD(LOG_UINT16, ch2, &ch[2])
LOG_ADD(LOG_UINT16, ch3, &ch[3])
LOG_ADD(LOG_UINT16, thrust, &extrxSetpoint.thrust)
LOG_ADD(LOG_FLOAT, roll, &extrxSetpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &extrxSetpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &extrxSetpoint.attitude.yaw)
LOG_GROUP_STOP(extrx)
#endif
