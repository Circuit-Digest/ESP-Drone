/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * sensors.h - Sensors interface
 */
#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "stabilizer_types.h"

typedef enum { ACC_MODE_PROPTEST, ACC_MODE_FLIGHT } accModes;

void sensorsInit(void);
bool sensorsTest(void);
bool sensorsAreCalibrated(void);

/**
 * More extensive test of the sensors
 */
bool sensorsManufacturingTest(void);

// For legacy control
void sensorsAcquire(sensorData_t *sensors, const uint32_t tick);

/**
 * This function should block and unlock at 1KhZ
 */
void sensorsWaitDataReady(void);

// Allows individual sensor measurement
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(baro_t *baro);

/**
 * Set acc mode, one of accModes enum
 */
void sensorsSetAccMode(accModes accMode);

#endif //__SENSORS_H__
