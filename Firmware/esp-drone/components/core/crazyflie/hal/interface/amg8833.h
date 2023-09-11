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
 * amg8833.h - Functions for interfacing AMG8833 thermal sensor
 * Reference : https://github.com/adafruit/Adafruit_AMG88xx
 */
#ifndef __AMG8833_H__
#define __AMG8833_H__

#include <stdint.h>
#include <stdbool.h>

#include "i2cdev.h"
#include "task.h"
#include "num.h"


#define MINTEMP                        22
#define MAXTEMP                        34

#define AMG_I2C_CHUNKSIZE              32
#define AMG88xx_PIXEL_ARRAY_SIZE       64

#define AMG88xx_ADDRESS              0x68
/************** Registers ***************/
#define AMG88xx_PCTL                 0x00
#define AMG88xx_RST                  0x01
#define AMG88xx_FPSC                 0x02
#define AMG88xx_INTC                 0x03
#define AMG88xx_STAT                 0x04
#define AMG88xx_SCLR                 0x05
#define AMG88xx_AVE                  0x07
#define AMG88xx_INTHL                0x08
#define AMG88xx_INTHH                0x09
#define AMG88xx_INTLL                0x0A
#define AMG88xx_INTLH                0x0B
#define AMG88xx_IHYSL                0x0C
#define AMG88xx_IHYSH                0x0D
#define AMG88xx_TTHL                 0x0E
#define AMG88xx_TTHH                 0x0F
#define AMG88xx_INT_OFFSET           0x10
#define AMG88xx_PIXEL_OFFSET         0x80
/************* Power modes *************/
#define AMG88xx_NORMAL_MODE          0x00
#define	AMG88xx_SLEEP_MODE           0x01
#define AMG88xx_STAND_BY_60          0x20
#define AMG88xx_STAND_BY_10          0x21
/*********** Software resets ***********/
#define AMG88xx_FLAG_RESET           0x30
#define AMG88xx_INITIAL_RESET        0x3F
/************* Frame rates *************/
#define AMG88xx_FPS_10               0x00
#define AMG88xx_FPS_1                0x01
/********** Interrupt Enables **********/
#define AMG88xx_INT_DISABLED         0x00
#define AMG88xx_INT_ENABLED          0x01
/********** Interrupt modes ************/
#define AMG88xx_DIFFERENCE           0x00
#define AMG88xx_ABSOLUTE_VALUE       0x01

typedef struct {
  uint8_t devAddr;
  I2C_Dev *I2Cx;
} AMG8833_Dev_t;

typedef AMG8833_Dev_t *AMG8833_DEV;

// Initiate thermal sensor
bool begin(AMG8833_Dev_t *dev, I2C_Dev *I2Cx);

// Data capture
void readPixels(AMG8833_Dev_t *dev, float *buf, uint8_t size);
float readThermistor(AMG8833_Dev_t *dev);

// Interrupts
bool enableInterrupt(AMG8833_Dev_t *dev);
bool disableInterrupt(AMG8833_Dev_t *dev);
void setInterruptMode(AMG8833_Dev_t *dev, uint8_t mode);
void getInterrupt(AMG8833_Dev_t *dev, uint8_t *buf, uint8_t size);
void clearInterrupt(AMG8833_Dev_t *dev);
// This will automatically set hysteresis to 95% of the high value
void setInterruptLevels_N(AMG8833_Dev_t *dev, float high, float low);
// This will manually set hysteresis
void setInterruptLevels_H(AMG8833_Dev_t *dev, float high, float low, float hysteresis);

// Modes
void setMovingAverageMode(AMG8833_Dev_t *dev, bool mode);

// Read operations
uint8_t read8(AMG8833_Dev_t *dev, uint8_t reg);
void read(AMG8833_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t num);

// Write operations
bool write8(AMG8833_Dev_t *dev, uint16_t reg, uint8_t value);
void write(AMG8833_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t num);

// Supportive calculations
float signedMag12ToFloat(uint16_t val);
float int12ToFloat(uint16_t val);
uint8_t min(uint8_t a, uint8_t b);

#endif /* __AMG8833_H__ */
