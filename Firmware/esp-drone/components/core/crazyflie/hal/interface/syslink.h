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
 * syslink.h: Implementation of the link between MCU
 */

#ifndef __SYSLINK_H__
#define __SYSLINK_H__

#include <stdbool.h>

#define SYSLINK_MTU 64

#define CRTP_START_BYTE  0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

// Defined packet types
#define SYSLINK_GROUP_MASK    0xF0

#define SYSLINK_RADIO_GROUP         0x00
#define SYSLINK_RADIO_RAW           0x00
#define SYSLINK_RADIO_CHANNEL       0x01
#define SYSLINK_RADIO_DATARATE      0x02
#define SYSLINK_RADIO_CONTWAVE      0x03
#define SYSLINK_RADIO_RSSI          0x04
#define SYSLINK_RADIO_ADDRESS       0x05
#define SYSLINK_RADIO_RAW_BROADCAST 0x06
#define SYSLINK_RADIO_POWER         0x07
#define SYSLINK_RADIO_P2P           0x08
#define SYSLINK_RADIO_P2P_ACK       0x09
#define SYSLINK_RADIO_P2P_BROADCAST 0x0A

#define SYSLINK_PM_GROUP              0x10
#define SYSLINK_PM_SOURCE             0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF    0x11
#define SYSLINK_PM_BATTERY_VOLTAGE    0x12
#define SYSLINK_PM_BATTERY_STATE      0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14

#define SYSLINK_OW_GROUP    0x20
#define SYSLINK_OW_SCAN     0x20
#define SYSLINK_OW_GETINFO  0x21
#define SYSLINK_OW_READ     0x22
#define SYSLINK_OW_WRITE    0x23

typedef struct _SyslinkPacket
{
  uint8_t type;
  uint8_t length;
  char data[SYSLINK_MTU];
} __attribute__((packed)) SyslinkPacket;

typedef enum
{
  waitForFirstStart,
  waitForSecondStart,
  waitForType,
  waitForLength,
  waitForData,
  waitForChksum1,
  waitForChksum2
} SyslinkRxState;


void syslinkInit();
bool syslinkTest();
int syslinkSendPacket(SyslinkPacket *slp);

#endif
