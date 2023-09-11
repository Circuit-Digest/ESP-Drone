/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 */
#include <math.h>

#include "sensfusion6.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"

//#define MADWICK_QUATERNION_IMU

#ifdef MADWICK_QUATERNION_IMU
  #define BETA_DEF     0.01f    // 2 * proportional gain
#else // MAHONY_QUATERNION_IMU
    #define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
    #define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain
#endif

#ifdef MADWICK_QUATERNION_IMU
  float beta = BETA_DEF;     // 2 * proportional gain (Kp)
#else // MAHONY_QUATERNION_IMU
  float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
  float integralFBx = 0.0f;
  float integralFBy = 0.0f;
  float integralFBz = 0.0f;  // integral error terms scaled by Ki
#endif

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

static float gravX, gravY, gravZ; // Unit vector in the estimated gravity direction

// The acc in Z for static position (g)
// Set on first update, assuming we are in a static position since the sensors were just calibrates.
// This value will be better the more level the copter is at calibration time
static float baseZacc = 1.0;

static bool isInit;

static bool isCalibrated = false;

static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static float sensfusion6GetAccZ(const float ax, const float ay, const float az);
static void estimatedGravityDirection(float* gx, float* gy, float* gz);

// TODO: Make math util file
static float invSqrt(float x);

void sensfusion6Init()
{
  if(isInit)
    return;

  isInit = true;
}

bool sensfusion6Test(void)
{
  return isInit;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  sensfusion6UpdateQImpl(gx, gy, gz, ax, ay, az, dt);
  estimatedGravityDirection(&gravX, &gravY, &gravZ);

  if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}

#ifdef MADWICK_QUATERNION_IMU
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy ,_8qx, _8qy, qwqw, qxqx, qyqy, qzqz;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz);
  qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy);
  qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx);
  qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2qw = 2.0f * qw;
    _2qx = 2.0f * qx;
    _2qy = 2.0f * qy;
    _2qz = 2.0f * qz;
    _4qw = 4.0f * qw;
    _4qx = 4.0f * qx;
    _4qy = 4.0f * qy;
    _8qx = 8.0f * qx;
    _8qy = 8.0f * qy;
    qwqw = qw * qw;
    qxqx = qx * qx;
    qyqy = qy * qy;
    qzqz = qz * qz;

    // Gradient decent algorithm corrective step
    s0 = _4qw * qyqy + _2qy * ax + _4qw * qxqx - _2qx * ay;
    s1 = _4qx * qzqz - _2qz * ax + 4.0f * qwqw * qx - _2qw * ay - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * az;
    s2 = 4.0f * qwqw * qy + _2qw * ax + _4qy * qzqz - _2qz * ay - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * az;
    s3 = 4.0f * qxqx * qz - _2qx * ax + 4.0f * qyqy * qz - _2qy * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  qw += qDot1 * dt;
  qx += qDot2 * dt;
  qy += qDot3 * dt;
  qz += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}
#else // MAHONY_QUATERNION_IMU
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * M_PI_F / 180;
  gy = gy * M_PI_F / 180;
  gz = gz * M_PI_F / 180;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = qx * qz - qw * qy;
    halfvy = qw * qx + qy * qz;
    halfvz = qw * qw - 0.5f + qz * qz;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = qw;
  qb = qx;
  qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz);
  qx += (qa * gx + qc * gz - qz * gy);
  qy += (qa * gy - qb * gz + qz * gx);
  qz += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}
#endif

void sensfusion6GetQuaternion(float* q_x, float* q_y, float* q_z, float* q_w)
{
  *q_x = qx;
  *q_y = qy;
  *q_z = qz;
  *q_w = qw;
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx = gravX;
  float gy = gravY;
  float gz = gravZ;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  *yaw = atan2f(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 180 / M_PI_F;
  *pitch = asinf(gx) * 180 / M_PI_F; //Pitch seems to be inverted
  *roll = atan2f(gy, gz) * 180 / M_PI_F;
}

float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

float sensfusion6GetInvThrustCompensationForTilt()
{
  // Return the z component of the estimated gravity direction
  // (0, 0, 1) dot G
  return gravZ;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static float sensfusion6GetAccZ(const float ax, const float ay, const float az)
{
  // return vertical acceleration
  // (A dot G) / |G|,  (|G| = 1) -> (A dot G)
  return (ax * gravX + ay * gravY + az * gravZ);
}

static void estimatedGravityDirection(float* gx, float* gy, float* gz)
{
  *gx = 2 * (qx * qz - qw * qy);
  *gy = 2 * (qw * qx + qy * qz);
  *gz = qw * qw - qx * qx - qy * qy + qz * qz;
}

LOG_GROUP_START(sensfusion6)
  LOG_ADD(LOG_FLOAT, qw, &qw)
  LOG_ADD(LOG_FLOAT, qx, &qx)
  LOG_ADD(LOG_FLOAT, qy, &qy)
  LOG_ADD(LOG_FLOAT, qz, &qz)
  LOG_ADD(LOG_FLOAT, gravityX, &gravX)
  LOG_ADD(LOG_FLOAT, gravityY, &gravY)
  LOG_ADD(LOG_FLOAT, gravityZ, &gravZ)
  LOG_ADD(LOG_FLOAT, accZbase, &baseZacc)
  LOG_ADD(LOG_UINT8, isInit, &isInit)
  LOG_ADD(LOG_UINT8, isCalibrated, &isCalibrated)
LOG_GROUP_STOP(sensfusion6)

PARAM_GROUP_START(sensfusion6)
#ifdef MADWICK_QUATERNION_IMU
PARAM_ADD(PARAM_FLOAT, beta, &beta)
#else // MAHONY_QUATERNION_IMU
PARAM_ADD(PARAM_FLOAT, kp, &twoKp)
PARAM_ADD(PARAM_FLOAT, ki, &twoKi)
#endif
PARAM_ADD(PARAM_FLOAT, baseZacc, &baseZacc)
PARAM_GROUP_STOP(sensfusion6)
