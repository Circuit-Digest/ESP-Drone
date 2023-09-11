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
 * imu_types.h - Types used by IMU and releted functions.
 */
#ifndef IMU_TYPES_H_
#define IMU_TYPES_H_

 typedef union {
   struct {
         int16_t x;
         int16_t y;
         int16_t z;
   };
   int16_t axis[3];
 } Axis3i16;

 typedef union {
   struct {
         int32_t x;
         int32_t y;
         int32_t z;
   };
   int32_t axis[3];
 } Axis3i32;

 typedef union {
   struct {
         int64_t x;
         int64_t y;
         int64_t z;
   };
   int64_t axis[3];
 } Axis3i64;

 typedef union {
   struct {
         float x;
         float y;
         float z;
   };
   float axis[3];
 } Axis3f;

#endif /* IMU_TYPES_H_ */
