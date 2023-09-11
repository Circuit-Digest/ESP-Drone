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
 * configblock.c - Simple static implementation of the config block
 */
#define DEBUG_MODULE "CFGBLK"

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#include "config.h"
#include "configblock.h"
#include "debug_cf.h"

/* Internal format of the config block */
#define MAGIC 0x43427830
#define VERSION 0
struct configblock_s {
  /* header */
  uint32_t magic;
  uint8_t  version;
  /* Content */
  uint8_t radioChannel;
  uint8_t radioSpeed;
  float calibPitch;
  float calibRoll;
  /* Simple modulo 256 checksum */
  uint8_t cksum;
} __attribute__((__packed__));

static struct configblock_s *configblock;

static bool cb_ok=false;

static uint8_t calculate_cksum(void* data, size_t len)
{
  unsigned char* c = data;
  int i;
  unsigned char cksum=0;
  
  for (i=0; i<len; i++)
    cksum += *(c++);

  return cksum;
}

int configblockInit(void)
{
  configblock = (void*)CONFIG_BLOCK_ADDRESS;

  //Verify the config block
  if (configblock->magic!=MAGIC || configblock->version!= VERSION || 
      calculate_cksum(configblock, sizeof(*configblock)) )
  {
    DEBUG_PRINT("Verification [FAIL]\n");
    return -1;
  }
  else
  {
    DEBUG_PRINT("v%d, verification [OK]\n", configblock->version);
    cb_ok = true;
  }

  
  return 0;
}

bool configblockTest(void)
{
  return true;
}

/* Static accessors */
int configblockGetRadioChannel(void)
{
  if (cb_ok)
    return configblock->radioChannel;
  else
    return RADIO_CHANNEL;
}

int configblockGetRadioSpeed(void)
{
  if (cb_ok)
    return configblock->radioSpeed;
  else
    return RADIO_DATARATE;
}

float configblockGetCalibPitch(void)
{
  if (cb_ok)
    return configblock->calibPitch;
  else
    return 0;
}

float configblockGetCalibRoll(void)
{
  if (cb_ok)
    return configblock->calibRoll;
  else
    return 0;
}

