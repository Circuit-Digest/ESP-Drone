/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "crtp_commander.h"

#include "commander.h"
#include "param.h"
#include "crtp.h"
#include "num.h"
#include "quatcompress.h"
#include "FreeRTOS.h"
#include "cfassert.h"

/* The generic commander format contains a packet type and data that has to be
 * decoded into a setpoint_t structure. The aim is to make it future-proof
 * by easily allowing the addition of new packets for future use cases.
 *
 * The packet format is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * The type is defined bellow together with a decoder function that should take
 * the data buffer in and fill up a setpoint_t structure.
 * The maximum data size is 29 bytes.
 */

/* To add a new packet:
 *   1 - Add a new type in the packetType_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the packetDecoders array.
 *   4 - Create a new params group for your handler if necessary
 *   5 - Pull-request your change :-)
 */

typedef void (*packetDecoder_t)(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen);

/* ---===== 1 - packetType_e enum =====--- */
enum packet_type {
  stopType          = 0,
  velocityWorldType = 1,
  zDistanceType     = 2,
  cppmEmuType       = 3,
  altHoldType       = 4,
  hoverType         = 5,
  fullStateType     = 6,
  positionType      = 7,
};

/* ---===== 2 - Decoding functions =====--- */
/* The setpoint structure is reinitialized to 0 before being passed to the
 * functions
 */

/* stopDecoder
 * Keeps setpoint to 0: stops the motors and fall
 */
static void stopDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  return;
}

/* velocityDecoder
 * Set the Crazyflie velocity in the world coordinate system
 */
struct velocityPacket_s {
  float vx;        // m in the world frame of reference
  float vy;        // ...
  float vz;        // ...
  float yawrate;  // deg/s
} __attribute__((packed));
static void velocityDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct velocityPacket_s *values = data;

  ASSERT(datalen == sizeof(struct velocityPacket_s));

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;

  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;
  setpoint->velocity.z = values->vz;

  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = -values->yawrate;
}

/* zDistanceDecoder
 * Set the Crazyflie absolute height and roll/pitch angles
 */
struct zDistancePacket_s {
  float roll;            // deg
  float pitch;           // ...
  float yawrate;         // deg/s
  float zDistance;        // m in the world frame of reference
} __attribute__((packed));
static void zDistanceDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct zDistancePacket_s *values = data;


  ASSERT(datalen == sizeof(struct zDistancePacket_s));

  setpoint->mode.z = modeAbs;

  setpoint->position.z = values->zDistance;


  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = -values->yawrate;


  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;

  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch;
}

/* cppmEmuDecoder
 * CRTP packet containing an emulation of CPPM channels
 * Channels have a range of 1000-2000 with a midpoint of 1500
 * Supports the ordinary RPYT channels plus up to MAX_AUX_RC_CHANNELS auxiliary channels.
 * Auxiliary channels are optional and transmitters do not have to transmit all the data
 * unless a given channel is actually in use (numAuxChannels must be set accordingly)
 *
 * Current aux channel assignments:
 * - AuxChannel0: set high to enable self-leveling, low to disable
 */
#define MAX_AUX_RC_CHANNELS 10

static float s_CppmEmuRollMaxRateDps = 720.0f; // For rate mode
static float s_CppmEmuPitchMaxRateDps = 720.0f; // For rate mode
static float s_CppmEmuRollMaxAngleDeg = 50.0f; // For level mode
static float s_CppmEmuPitchMaxAngleDeg = 50.0f; // For level mode
static float s_CppmEmuYawMaxRateDps = 400.0f; // Used regardless of flight mode

struct cppmEmuPacket_s {
  struct {
      uint8_t numAuxChannels : 4;   // Set to 0 through MAX_AUX_RC_CHANNELS
      uint8_t reserved : 4;
  } hdr;
  uint16_t channelRoll;
  uint16_t channelPitch;
  uint16_t channelYaw;
  uint16_t channelThrust;
  uint16_t channelAux[MAX_AUX_RC_CHANNELS];
} __attribute__((packed));

static inline float getChannelUnitMultiplier(uint16_t channelValue, uint16_t channelMidpoint, uint16_t channelRange)
{
  // Compute a float from -1 to 1 based on the RC channel value, midpoint, and total range magnitude
  return ((float)channelValue - (float)channelMidpoint) / (float)channelRange;
}

static void cppmEmuDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  bool isSelfLevelEnabled = true;

  ASSERT(datalen >= 9); // minimum 9 bytes expected - 1byte header + four 2byte channels
  const struct cppmEmuPacket_s *values = data;
  ASSERT(datalen == 9 + (2*values->hdr.numAuxChannels)); // Total size is 9 + number of active aux channels

  // Aux channel 0 is reserved for enabling/disabling self-leveling
  // If it's in use, check and see if it's set and enable self-leveling.
  // If aux channel 0 is not in use, default to self-leveling enabled.
  isSelfLevelEnabled = !(values->hdr.numAuxChannels >= 1 && values->channelAux[0] < 1500);

  // Set the modes

  // Position is disabled
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;

  // Yaw is always velocity
  setpoint->mode.yaw = modeVelocity;

  // Roll/Pitch mode is either velocity or abs based on isSelfLevelEnabled
  setpoint->mode.roll = isSelfLevelEnabled ? modeAbs : modeVelocity;
  setpoint->mode.pitch = isSelfLevelEnabled ? modeAbs : modeVelocity;

  // Rescale the CPPM values into angles to build the setpoint packet
  if(isSelfLevelEnabled)
  {
    setpoint->attitude.roll = -1 * getChannelUnitMultiplier(values->channelRoll, 1500, 500) * s_CppmEmuRollMaxAngleDeg; // roll inverted
    setpoint->attitude.pitch = -1 * getChannelUnitMultiplier(values->channelPitch, 1500, 500) * s_CppmEmuPitchMaxAngleDeg; // pitch inverted
  }
  else
  {
    setpoint->attitudeRate.roll = -1 * getChannelUnitMultiplier(values->channelRoll, 1500, 500) * s_CppmEmuRollMaxRateDps; // roll inverted
    setpoint->attitudeRate.pitch = -1 * getChannelUnitMultiplier(values->channelPitch, 1500, 500) * s_CppmEmuPitchMaxRateDps; // pitch inverted
  }

  setpoint->attitudeRate.yaw = -1 * getChannelUnitMultiplier(values->channelYaw, 1500, 500) * s_CppmEmuYawMaxRateDps; // yaw inverted
  setpoint->thrust = getChannelUnitMultiplier(values->channelThrust, 1000, 1000) * (float)UINT16_MAX; // Thrust is positive only - uses the full 1000-2000 range

  // Make sure thrust isn't negative
  if(setpoint->thrust < 0)
  {
    setpoint->thrust = 0;
  }
}

/* altHoldDecoder
 * Set the Crazyflie vertical velocity and roll/pitch angle
 */
struct altHoldPacket_s {
  float roll;            // rad
  float pitch;           // ...
  float yawrate;         // deg/s
  float zVelocity;       // m/s in the world frame of reference
} __attribute__((packed));
static void altHoldDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct altHoldPacket_s *values = data;

  ASSERT(datalen == sizeof(struct altHoldPacket_s));


  setpoint->mode.z = modeVelocity;

  setpoint->velocity.z = values->zVelocity;


  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = -values->yawrate;


  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;

  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch;
}

/* hoverDecoder
 * Set the Crazyflie absolute height and velocity in the body coordinate system
 */
struct hoverPacket_s {
  float vx;           // m/s in the body frame of reference
  float vy;           // ...
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed));
static void hoverDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct hoverPacket_s *values = data;

  ASSERT(datalen == sizeof(struct hoverPacket_s));

  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;

  setpoint->velocity_body = true;
}

struct fullStatePacket_s {
  int16_t x;         // position - mm
  int16_t y;
  int16_t z;
  int16_t vx;        // velocity - mm / sec
  int16_t vy;
  int16_t vz;
  int16_t ax;        // acceleration - mm / sec^2
  int16_t ay;
  int16_t az;
  int32_t quat;      // compressed quaternion, see quatcompress.h
  int16_t rateRoll;  // angular velocity - milliradians / sec
  int16_t ratePitch; //  (NOTE: limits to about 5 full circles per sec.
  int16_t rateYaw;   //   may not be enough for extremely aggressive flight.)
} __attribute__((packed));
static void fullStateDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct fullStatePacket_s *values = data;

  ASSERT(datalen == sizeof(struct fullStatePacket_s));

  #define UNPACK(x) \
  setpoint->mode.x = modeAbs; \
  setpoint->position.x = values->x / 1000.0f; \
  setpoint->velocity.x = (values->v ## x) / 1000.0f; \
  setpoint->acceleration.x = (values->a ## x) / 1000.0f; \

  UNPACK(x)
  UNPACK(y)
  UNPACK(z)
  #undef UNPACK

  float const millirad2deg = 180.0f / ((float)M_PI * 1000.0f);
  setpoint->attitudeRate.roll = millirad2deg * values->rateRoll;
  setpoint->attitudeRate.pitch = millirad2deg * values->ratePitch;
  setpoint->attitudeRate.yaw = millirad2deg * values->rateYaw;

  quatdecompress(values->quat, (float *)&setpoint->attitudeQuaternion.q0);
  setpoint->mode.quat = modeAbs;
  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
  setpoint->mode.yaw = modeDisable;
}

/* positionDecoder
 * Set the absolute postition and orientation
 */
 struct positionPacket_s {
   float x;     // Position in m
   float y;
   float z;
   float yaw;   // Orientation in degree
 } __attribute__((packed));
static void positionDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct positionPacket_s *values = data;

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;

  setpoint->position.x = values->x;
  setpoint->position.y = values->y;
  setpoint->position.z = values->z;


  setpoint->mode.yaw = modeAbs;

  setpoint->attitude.yaw = values->yaw;
}

 /* ---===== 3 - packetDecoders array =====--- */
const static packetDecoder_t packetDecoders[] = {
  [stopType]          = stopDecoder,
  [velocityWorldType] = velocityDecoder,
  [zDistanceType]     = zDistanceDecoder,
  [cppmEmuType]       = cppmEmuDecoder,
  [altHoldType]       = altHoldDecoder,
  [hoverType]         = hoverDecoder,
  [fullStateType]     = fullStateDecoder,
  [positionType]      = positionDecoder,
};

/* Decoder switch */
void crtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  static int nTypes = -1;

  ASSERT(pk->size > 0);

  if (nTypes<0) {
    nTypes = sizeof(packetDecoders)/sizeof(packetDecoders[0]);
  }

  uint8_t type = pk->data[0];

  memset(setpoint, 0, sizeof(setpoint_t));

  if (type<nTypes && (packetDecoders[type] != NULL)) {
    packetDecoders[type](setpoint, type, ((char*)pk->data)+1, pk->size-1);
  }
}

// Params for generic CRTP handlers

// CPPM Emulation commander
PARAM_GROUP_START(cmdrCPPM)
PARAM_ADD(PARAM_FLOAT, rateRoll, &s_CppmEmuRollMaxRateDps)
PARAM_ADD(PARAM_FLOAT, ratePitch, &s_CppmEmuPitchMaxRateDps)
PARAM_ADD(PARAM_FLOAT, rateYaw, &s_CppmEmuYawMaxRateDps)
PARAM_ADD(PARAM_FLOAT, angRoll, &s_CppmEmuRollMaxAngleDeg)
PARAM_ADD(PARAM_FLOAT, angPitch, &s_CppmEmuPitchMaxAngleDeg)
PARAM_GROUP_STOP(cmdrCPPM)
