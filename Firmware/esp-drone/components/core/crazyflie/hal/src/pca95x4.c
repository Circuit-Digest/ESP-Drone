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
 * pca95x4.c - Functions for interfacing PCA95X4 I2C GPIO extender
 */
#define DEBUG_MODULE "PCA95X4"

#include "i2cdev.h"

#include "pca95x4.h"

#include "debug_cf.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;

void pca95x4Init()
{
  i2cdevInit(I2C1_DEV);
  I2Cx = I2C1_DEV;
  devAddr = PCA95X4_DEFAULT_ADDRESS;
}

bool pca95x4Test()
{
  uint8_t tb;
  bool pass;

  // Test reading the config register
  pass = i2cdevReadByte(I2Cx, devAddr, PCA95X4_CONFIG_REG, &tb);

  return pass;
}

bool pca95x4ConfigOutput(uint32_t val) {
  bool pass;

  pass = i2cdevWriteByte(I2Cx, devAddr, PCA95X4_CONFIG_REG, val);
  return pass;
}

bool pca95x4SetOutput(uint32_t mask) {
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA95X4_OUTPUT_REG, &val);
  val |= mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA95X4_OUTPUT_REG, val);

  return pass;
}

bool pca95x4ClearOutput(uint32_t mask) {
  uint8_t val;
  bool pass;

  pass = i2cdevReadByte(I2Cx, devAddr, PCA95X4_OUTPUT_REG, &val);
  val &= ~mask;
  pass = i2cdevWriteByte(I2Cx, devAddr, PCA95X4_OUTPUT_REG, val);

  return pass;
}
