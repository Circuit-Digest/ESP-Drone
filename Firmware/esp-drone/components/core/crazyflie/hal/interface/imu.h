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
 *
 * imu.h - inertial measurement unit header file
 */
#ifndef IMU_H_
#define IMU_H_
#include <stdbool.h>
#include "filter.h"
#include "imu_types.h"

/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  4
/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

void imu6Init(void);
bool imu6Test(void);
bool imu6ManufacturingTest(void);
void imu6Read(Axis3f* gyro, Axis3f* acc);
void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut);
bool imu6IsCalibrated(void);
bool imuHasBarometer(void);
bool imuHasMangnetometer(void);



#endif /* IMU_H_ */
