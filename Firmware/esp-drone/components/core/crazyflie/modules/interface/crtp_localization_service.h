/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
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
 */

#ifndef _CRTP_LOCALIZATION_SERVICE_H_
#define _CRTP_LOCALIZATION_SERVICE_H_

#include "stabilizer_types.h"
//#include "pulse_processor.h"

/**
 * CRTP external position data struct
 */
struct CrtpExtPosition
{
  float x; // in m
  float y; // in m
  float z; // in m
} __attribute__((packed));

struct CrtpExtPose
{
  float x; // in m
  float y; // in m
  float z; // in m
  float qx;
  float qy;
  float qz;
  float qw;
} __attribute__((packed));

typedef enum
{
  RANGE_STREAM_FLOAT      = 0,
  RANGE_STREAM_FP16       = 1,
  LPS_SHORT_LPP_PACKET    = 2,
  EMERGENCY_STOP          = 3,
  EMERGENCY_STOP_WATCHDOG = 4,
  COMM_GNSS_NMEA           = 6,
  COMM_GNSS_PROPRIETARY    = 7,
  EXT_POSE                 = 8,
  EXT_POSE_PACKED          = 9,
  LH_ANGLE_STREAM          = 10,
  LH_PERSIST_DATA          = 11,
} locsrv_t;

// Set up the callback for the CRTP_PORT_LOCALIZATION
void locSrvInit(void);

// Send range in float. After 5 ranges it will send the packet.
void locSrvSendRangeFloat(uint8_t id, float range);
//void locSrvSendLighthouseAngle(int basestation, pulseProcessorResult_t* angles);

#endif /* _CRTP_LOCALIZATION_SERVICE_H_ */
