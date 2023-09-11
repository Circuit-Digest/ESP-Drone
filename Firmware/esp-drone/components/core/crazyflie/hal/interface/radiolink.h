/*
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
 * radiolink.c - Radio link layer
 */

#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdint.h>
#include <stdbool.h>
#include "syslink.h"

#define P2P_MAX_DATA_SIZE 60

typedef struct _P2PPacket
{
  uint8_t size;                         //< Size of data
  uint8_t rssi;                         //< Received Signal Strength Intensity
  union {
    struct {
      uint8_t port;                 //< Header selecting channel and port
      uint8_t data[P2P_MAX_DATA_SIZE]; //< Data
    };
    uint8_t raw[P2P_MAX_DATA_SIZE+1];  //< The full packet "raw"
  };
} __attribute__((packed)) P2PPacket;

typedef void (*P2PCallback)(P2PPacket *);

void radiolinkInit(void);
bool radiolinkTest(void);
void radiolinkSetChannel(uint8_t channel);
void radiolinkSetDatarate(uint8_t datarate);
void radiolinkSetAddress(uint64_t address);
void radiolinkSetPowerDbm(int8_t powerDbm);
void radiolinkSyslinkDispatch(SyslinkPacket *slp);
struct crtpLinkOperations * radiolinkGetLink();
bool radiolinkSendP2PPacketBroadcast(P2PPacket *p2pp);
void p2pRegisterCB(P2PCallback cb);


#endif //__RADIO_H__
