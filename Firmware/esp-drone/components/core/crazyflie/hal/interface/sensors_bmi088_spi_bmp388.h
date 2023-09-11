/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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

#ifndef __SENSORS_BMI088SPI_BMP388_H__
#define __SENSORS_BMI088SPI_BMP388_H__

#include "sensors.h"

void sensorsBmi088SpiBmp388Init(void);
bool sensorsBmi088SpiBmp388Test(void);
bool sensorsBmi088SpiBmp388AreCalibrated(void);
bool sensorsBmi088SpiBmp388ManufacturingTest(void);
void sensorsBmi088SpiBmp388Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsBmi088SpiBmp388WaitDataReady(void);
bool sensorsBmi088SpiBmp388ReadGyro(Axis3f *gyro);
bool sensorsBmi088SpiBmp388ReadAcc(Axis3f *acc);
bool sensorsBmi088SpiBmp388ReadMag(Axis3f *mag);
bool sensorsBmi088SpiBmp388ReadBaro(baro_t *baro);
void sensorsBmi088SpiBmp388SetAccMode(accModes accMode);
void sensorsBmi088SpiBmp388DataAvailableCallback(void);

#endif // __SENSORS_BMI088SPI_BMP388_H__