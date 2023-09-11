/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2017 Bitcraze AB
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
 * pca9555.c - Functions for interfacing PCA9555 I2C GPIO extender
 */
#define DEBUG_MODULE "PCA9555"

#include "i2cdev.h"

#include "pca9555.h"

#include "debug_cf.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;

void pca9555Init()
{
  i2cdevInit(I2C1_DEV);
  I2Cx = I2C1_DEV;
  devAddr = PCA9555_DEFAULT_ADDRESS;
}

/**
 * Reads the config registers and checks if there are any errors in reading
 * the registers
 */
bool pca9555Test()
{
  uint8_t tb;
  bool pass_set1, pass_set2;

  // Test reading the config registers
  pass_set1 = i2cdevReadByte(I2Cx, devAddr, PCA9555_CONFIG_REGA, &tb);
  pass_set2 = i2cdevReadByte(I2Cx, devAddr, PCA9555_CONFIG_REGB, &tb);

  return pass_set1 & pass_set2;
}

bool pca9555ConfigOutputRegA(uint32_t val)
{
  return i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REGA, val);
}

bool pca9555ConfigOutputRegB(uint32_t val)
{
  return i2cdevWriteByte(I2Cx, devAddr, PCA9555_CONFIG_REGB, val);
}

bool pca9555SetOutputRegA(uint32_t mask)
{
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val);
  val |= mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val);

  return pass;
}

bool pca9555SetOutputRegB(uint32_t mask)
{
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, &val);
  val |= mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, val);

  return pass;
}

bool pca9555ClearOutputRegA(uint32_t mask)
{
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, &val);
  val &= ~mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGA, val);

  return pass;
}

bool pca9555ClearOutputRegB(uint32_t mask)
{
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, &val);
  val &= ~mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA9555_OUTPUT_REGB, val);

  return pass;
}
