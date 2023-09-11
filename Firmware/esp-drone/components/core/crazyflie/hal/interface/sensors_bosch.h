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
 * sensors_bosch.h - Sensors interface
 */
#ifndef __SENSORS_BOSCH_H__
#define __SENSORS_BOSCH_H__

#include "sensors.h"

void sensorsBoschInit(void);
bool sensorsBoschTest(void);
bool sensorsBoschAreCalibrated(void);
bool sensorsBoschManufacturingTest(void);
void sensorsBoschAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsBoschWaitDataReady(void);
bool sensorsBoschReadGyro(Axis3f *gyro);
bool sensorsBoschReadAcc(Axis3f *acc);
bool sensorsBoschReadMag(Axis3f *mag);
bool sensorsBoschReadBaro(baro_t *baro);
void sensorsBoschSetAccMode(accModes accMode);
void sensorsBoschDataAvailableCallback(void);

#endif /* __SENSORS_BOSCH_H__ */
