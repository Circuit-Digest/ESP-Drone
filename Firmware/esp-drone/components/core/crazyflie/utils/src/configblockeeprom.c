/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
*
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
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
#include <string.h>

#include "config.h"
#include "debug_cf.h"
#include "i2cdev.h"
#include "configblock.h"
#include "eeprom.h"


/* Internal format of the config block */
#define MAGIC 0x43427830
#define VERSION 1
#define HEADER_SIZE_BYTES 5 // magic + version
#define OVERHEAD_SIZE_BYTES (HEADER_SIZE_BYTES + 1) // + cksum

// Old versions
struct configblock_v0_s {
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

// Current version
struct configblock_v1_s {
  /* header */
  uint32_t magic;
  uint8_t  version;
  /* Content */
  uint8_t radioChannel;
  uint8_t radioSpeed;
  float calibPitch;
  float calibRoll;
  uint8_t radioAddress_upper;
  uint32_t radioAddress_lower;
  /* Simple modulo 256 checksum */
  uint8_t cksum;
} __attribute__((__packed__));

// Set version 1 as current version
typedef struct configblock_v1_s configblock_t;

static configblock_t configblock;
static configblock_t configblockDefault =
{
    .magic = MAGIC,
    .version = VERSION,
    .radioChannel = RADIO_CHANNEL,
    .radioSpeed = RADIO_DATARATE,
    .calibPitch = 0.0,
    .calibRoll = 0.0,
    .radioAddress_upper = ((uint64_t)RADIO_ADDRESS >> 32),
    .radioAddress_lower = (RADIO_ADDRESS & 0xFFFFFFFFULL),
};

static const uint32_t configblockSizes[] =
{
  sizeof(struct configblock_v0_s),
  sizeof(struct configblock_v1_s),
};

static bool isInit = false;
static bool cb_ok = false;

static bool configblockCheckMagic(configblock_t *configblock);
static bool configblockCheckVersion(configblock_t *configblock);
static bool configblockCheckChecksum(configblock_t *configblock);
static bool configblockCheckDataIntegrity(uint8_t *data, uint8_t version);
static bool configblockWrite(configblock_t *configblock);
static bool configblockCopyToNewVersion(configblock_t *configblockSaved, configblock_t *configblockNew);

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
  if(isInit)
    return 0;

  i2cdevInit(I2C1_DEV);
  eepromInit(I2C1_DEV);

  // Because of strange behavior from I2C device during expansion port test
  // the first read needs to be discarded
  eepromTestConnection();

  if (eepromTestConnection())
  {
    if (eepromReadBuffer((uint8_t *)&configblock, 0, sizeof(configblock)))
    {
      //Verify the config block
      if (configblockCheckMagic(&configblock))
      {
        if (configblockCheckVersion(&configblock))
        {
          if (configblockCheckChecksum(&configblock))
          {
            // Everything is fine
            DEBUG_PRINTD("v%d, verification [OK]\n", configblock.version);
            cb_ok = true;
          }
          else
          {
            DEBUG_PRINTD("Verification [FAIL]\n");
            cb_ok = false;
          }
        }
        else // configblockCheckVersion
        {
          // Check data integrity of old version data
          if (configblock.version <= VERSION &&
              configblockCheckDataIntegrity((uint8_t *)&configblock, configblock.version))
          {
            // Not the same version, try to upgrade
            if (configblockCopyToNewVersion(&configblock, &configblockDefault))
            {
              // Write updated config block to eeprom
              if (configblockWrite(&configblock))
              {
                cb_ok = true;
              }
            }
          }
          else
          {
            // Can't copy old version due to bad data.
            cb_ok = false;
          }
        }
      }
    }
  }

  if (cb_ok == false)
  {
    // Copy default data to used structure.
    memcpy((uint8_t *)&configblock, (uint8_t *)&configblockDefault, sizeof(configblock));
    // Write default configuration to eeprom
    if (configblockWrite(&configblockDefault))
    {
      cb_ok = true;
    }
    else
    {
      return -1;
    }
  }

  isInit = true;

  return 0;
}

bool configblockTest(void)
{
  return eepromTest();
}

static bool configblockCheckMagic(configblock_t *configblock)
{
  return (configblock->magic == MAGIC);
}

static bool configblockCheckVersion(configblock_t *configblock)
{
  return (configblock->version == VERSION);
}

static bool configblockCheckChecksum(configblock_t *configblock)
{
  return (configblock->cksum == calculate_cksum(configblock, sizeof(configblock_t) - 1));
}

static bool configblockCheckDataIntegrity(uint8_t *data, uint8_t version)
{
  bool status = false;

  if (version == 0)
  {
    struct configblock_v0_s *v0 = ( struct configblock_v0_s *)data;
    status = (v0->cksum == calculate_cksum(data, sizeof(struct configblock_v0_s) - 1));
  }
  else if (version == 1)
  {
    struct configblock_v1_s *v1 = ( struct configblock_v1_s *)data;
    status = (v1->cksum == calculate_cksum(data, sizeof(struct configblock_v1_s) - 1));
  }

  return status;
}

static bool configblockWrite(configblock_t *configblock)
{
  // Write default configuration to eeprom
  configblock->cksum = calculate_cksum(configblock, sizeof(configblock_t) - 1);
  if (!eepromWriteBuffer((uint8_t *)configblock, 0, sizeof(configblock_t)))
  {
    return false;
  }

  return true;
}

static bool configblockCopyToNewVersion(configblock_t *configblockSaved, configblock_t *configblockNew)
{
  configblock_t configblockTmp;

  // Copy new data to temp config memory
  memcpy((uint8_t *)&configblockTmp, (uint8_t *)configblockNew, sizeof(configblock_t));

  if (configblockSaved->version <= VERSION &&
      sizeof(configblock_t) >= configblockSizes[configblockSaved->version])
  {
    // Copy old saved eeprom data to new structure
    memcpy((uint8_t *)&configblockTmp + HEADER_SIZE_BYTES,
           (uint8_t *)configblockSaved + HEADER_SIZE_BYTES,
           configblockSizes[configblockSaved->version] - OVERHEAD_SIZE_BYTES);
    // Copy updated block to saved structure
    memcpy((uint8_t *)configblockSaved, (uint8_t *)&configblockTmp, sizeof(configblock_t));
  }
  else
  {
    return false;
  }

  return true;
}

/* Static accessors */
int configblockGetRadioChannel(void)
{
  if (cb_ok)
    return configblock.radioChannel;
  else
    return RADIO_CHANNEL;
}

int configblockGetRadioSpeed(void)
{
  if (cb_ok)
    return configblock.radioSpeed;
  else
    return RADIO_DATARATE;
}

uint64_t configblockGetRadioAddress(void)
{
  if (cb_ok)
    return ((uint64_t)configblock.radioAddress_upper << 32) | (uint64_t)configblock.radioAddress_lower;
  else
    return RADIO_ADDRESS;
}

float configblockGetCalibPitch(void)
{
  if (cb_ok)
    return configblock.calibPitch;
  else
    return 0;
}

float configblockGetCalibRoll(void)
{
  if (cb_ok)
    return configblock.calibRoll;
  else
    return 0;
}
