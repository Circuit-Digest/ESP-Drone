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
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __SENSORS_MPU9250_LPS25H_H__
#define __SENSORS_MPU9250_LPS25H_H__

#include "sensors.h"

void sensorsMpu6050Hmc5883lMs5611Init(void);
bool sensorsMpu6050Hmc5883lMs5611Test(void);
bool sensorsMpu6050Hmc5883lMs5611AreCalibrated(void);
bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void);
void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro);
bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc);
bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag);
bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro);
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode);

#endif // __SENSORS_MPU9250_LPS25H_H__