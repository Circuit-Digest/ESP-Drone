/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * sitAw.h - Implementation of situation awareness.
 */

#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#include "log.h"
#include "param.h"
#include "trigger.h"
#include "sitaw.h"
#include "commander.h"
#include "stabilizer.h"
#include "motors.h"

/* Trigger object used to detect Free Fall situation. */
static trigger_t sitAwFFAccWZ;

/* Trigger object used to detect At Rest situation. */
static trigger_t sitAwARAccZ;

/* Trigger object used to detect Tumbled situation. */
static trigger_t sitAwTuAcc;

#if defined(SITAW_ENABLED)

#if defined(SITAW_LOG_ENABLED) /* Enable the log group. */
LOG_GROUP_START(sitAw)
#if defined(SITAW_FF_LOG_ENABLED) /* Log trigger variables for Free Fall detection. */
LOG_ADD(LOG_UINT32, FFAccWZTestCounter, &sitAwFFAccWZ.testCounter)
LOG_ADD(LOG_UINT8, FFAccWZDetected, &sitAwFFAccWZ.released)
#endif
#if defined(SITAW_AR_LOG_ENABLED) /* Log trigger variables for At Rest detection. */
LOG_ADD(LOG_UINT32, ARTestCounter, &sitAwARAccZ.testCounter)
LOG_ADD(LOG_UINT8, ARDetected, &sitAwARAccZ.released)
#endif
#if defined(SITAW_TU_LOG_ENABLED) /* Log trigger variables for Tumbled detection. */
LOG_ADD(LOG_UINT32, TuTestCounter, &sitAwTuAcc.testCounter)
LOG_ADD(LOG_UINT8, TuDetected, &sitAwTuAcc.released)
#endif
#if defined(SITAW_LOG_ALL_DETECT_ENABLED) /* Log all the 'Detected' flags. */
LOG_ADD(LOG_UINT8, FFAccWZDetected, &sitAwFFAccWZ.released)
LOG_ADD(LOG_UINT8, ARDetected, &sitAwARAccZ.released)
LOG_ADD(LOG_UINT8, TuDetected, &sitAwTuAcc.released)
#endif
LOG_GROUP_STOP(sitAw)
#endif /* SITAW_LOG_ENABLED */

#if defined(SITAW_PARAM_ENABLED) /* Enable the param group. */
PARAM_GROUP_START(sitAw)
#if defined(SITAW_FF_PARAM_ENABLED) /* Param variables for Free Fall detection. */
PARAM_ADD(PARAM_UINT8, FFActive, &sitAwFFAccWZ.active)
PARAM_ADD(PARAM_UINT32, FFTriggerCount, &sitAwFFAccWZ.triggerCount)
PARAM_ADD(PARAM_FLOAT, FFaccWZ, &sitAwFFAccWZ.threshold)
#endif
#if defined(SITAW_AR_PARAM_ENABLED) /* Param variables for At Rest detection. */
PARAM_ADD(PARAM_UINT8, ARActive, &sitAwARAccZ.active)
PARAM_ADD(PARAM_UINT32, ARTriggerCount, &sitAwARAccZ.triggerCount)
PARAM_ADD(PARAM_FLOAT, ARaccZ, &sitAwARAccZ.threshold)
#endif
#if defined(SITAW_TU_PARAM_ENABLED) /* Param variables for Tumbled detection. */
PARAM_ADD(PARAM_UINT8, TuActive, &sitAwTuAcc.active)
PARAM_ADD(PARAM_UINT32, TuTriggerCount, &sitAwTuAcc.triggerCount)
PARAM_ADD(PARAM_FLOAT, TuAcc, &sitAwTuAcc.threshold)
#endif
PARAM_GROUP_STOP(sitAw)
#endif /* SITAW_PARAM_ENABLED */

#endif /* SITAW_ENABLED */

// forward declaration of private functions
#ifdef SITAW_FF_ENABLED
static void sitAwFFTest(float accWZ, float accMag);
#endif
#ifdef SITAW_AR_ENABLED
static void sitAwARTest(float accX, float accY, float accZ);
#endif
#ifdef SITAW_TU_ENABLED
static void sitAwTuTest(float accz);
#endif

static void sitAwPostStateUpdateCallOut(const sensorData_t *sensorData,
                                        const state_t *state)
{
  /* Code that shall run AFTER each attitude update, should be placed here. */

#if defined(SITAW_ENABLED)
#ifdef SITAW_FF_ENABLED
  float accMAG = (sensorData->acc.x*sensorData->acc.x) +
                 (sensorData->acc.y*sensorData->acc.y) +
                 (sensorData->acc.z*sensorData->acc.z);

  /* Test values for Free Fall detection. */
  sitAwFFTest(state->acc.z, accMAG);
#endif
#ifdef SITAW_TU_ENABLED
  /* check if we actually fly */
  int sumRatio = 0;
  for (int i = 0; i < NBR_OF_MOTORS; ++i) {
    sumRatio += motorsGetRatio(i);
  }
  bool isFlying = sumRatio > SITAW_TU_IN_FLIGHT_THRESHOLD;
  if (isFlying) {
    /* Test values for Tumbled detection. */
    sitAwTuTest(sensorData->acc.z);
  }
#endif
#ifdef SITAW_AR_ENABLED
/* Test values for At Rest detection. */
  sitAwARTest(sensorData->acc.x, sensorData->acc.y, sensorData->acc.z);
#endif
#endif
}

static void sitAwPreThrustUpdateCallOut(setpoint_t *setpoint)
{
  /* Code that shall run BEFORE each thrust distribution update, should be placed here. */

#if defined(SITAW_ENABLED)
#ifdef SITAW_TU_ENABLED
      if(sitAwTuDetected()) {
        /* Kill the thrust to the motors if a Tumbled situation is detected. */
        stabilizerSetEmergencyStop();
      }
#endif

#ifdef SITAW_FF_ENABLED
      /* Force altHold mode if free fall is detected.
         FIXME: Needs a flying/landing state (as soon as althold is enabled,
                                              we are not freefalling anymore)
       */
      if(sitAwFFDetected() && !sitAwTuDetected()) {
        setpoint->mode.z = modeVelocity;
        setpoint->velocity.z = 0;
      }
#endif
#endif
}

/**
 * Update setpoint according to current situation
 *
 * Called by the stabilizer after state and setpoint update. This function
 * should update the setpoint accordig to the current state situation
 */
void sitAwUpdateSetpoint(setpoint_t *setpoint, const sensorData_t *sensorData,
                                               const state_t *state)
{
  sitAwPostStateUpdateCallOut(sensorData, state);
  sitAwPreThrustUpdateCallOut(setpoint);
}

#ifdef SITAW_FF_ENABLED
/**
 * Initialize the Free Fall detection.
 *
 * See the sitAwFFTest() function for details.
 */
void sitAwFFInit(void)
{
  triggerInit(&sitAwFFAccWZ, triggerFuncIsLE, SITAW_FF_THRESHOLD, SITAW_FF_TRIGGER_COUNT);
  triggerActivate(&sitAwFFAccWZ, true);
}

/**
 * Test values for a Free Fall situation.
 *
 * A free fall situation is considered identified when the vertical
 * acceleration of the crazyflie (regardless of orientation - given by
 * AccWZ) is approaching -1 (AccWZ is 0 when crazyflie is at rest). We
 * will look for when AccWZ is within SITAW_FF_THRESHOLD of -1.
 *
 * At the same time, there should be no other accelerations experienced
 * by the crazyflie.

 * This can be checked by looking at the accMAG (total acceleration). If
 * the accMAG is approaching 0, there are no other accelerations than accWZ.
 * This helps to distinguish free fall situations from other movements
 * such as shaking.
 *
 * @param accWZ  Vertical acceleration (regardless of orientation)
 * @param accMAG All experienced accelerations.
 */
void sitAwFFTest(float accWZ, float accMAG)
{
  /* Check that the total acceleration is close to zero. */
  if(fabs(accMAG) > SITAW_FF_THRESHOLD) {
    /* If the total acceleration deviates from 0, this is not a free fall situation. */
    triggerReset(&sitAwFFAccWZ);
  } else {

    /**
     * AccWZ approaches -1 in free fall. Check that the value stays within
     * SITAW_FF_THRESHOLD of -1 for the triggerCount specified.
     */
    triggerTestValue(&sitAwFFAccWZ, fabs(accWZ + 1));
  }
}

/**
 * Check if a Free Fall situation has been detected.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwFFDetected(void)
{
  return sitAwFFAccWZ.released;
}
#endif

#ifdef SITAW_AR_ENABLED
/**
 * Initialize the At Rest detection.
 *
 * See the sitAwARTest() function for details.
 */
void sitAwARInit(void)
{
  triggerInit(&sitAwARAccZ, triggerFuncIsLE, SITAW_AR_THRESHOLD, SITAW_AR_TRIGGER_COUNT);
  triggerActivate(&sitAwARAccZ, true);
}

/**
 * Test values for an At Rest situation.
 *
 * An At Rest situation is considered identified when the crazyflie is
 * placed on its feet (accZ = 1) and with no horizontal accelerations
 * (accX = accY = 0).
 *
 * Since there is always some minor noise in the measurements, we use
 * a margin of SITAW_AR_THRESHOLD from the ideal values. Since this function
 * does not check for thrust, the SITAW_AR_THRESHOLD is assumed to be set
 * sufficiently close to the absolute resting values so that these values cannot
 * be achieved (over time) during hovering or flight.
 *
 * @param accX   Horizontal X acceleration (when crazyflie is placed on its feet)
 * @param accY   Horizontal Y acceleration (when crazyflie is placed on its feet)
 * @param accZ   Vertical Z acceleration (when crazyflie is placed on its feet)
 */
void sitAwARTest(float accX, float accY, float accZ)
{
  /* Check that there are no horizontal accelerations. At rest, these are 0. */
  if((fabs(accX) > SITAW_AR_THRESHOLD) || (fabs(accY) > SITAW_AR_THRESHOLD)) {
    /* If the X or Y accelerations are different than 0, the crazyflie is not at rest. */
    triggerReset(&sitAwARAccZ);
  }

  /**
   * If the test above indicates that there are no horizontal movements, test the
   * vertical acceleration value against the trigger.
   *
   * The vertical acceleration must be close to 1, but is allowed to oscillate slightly
   * around 1. Testing that the deviation from 1 stays within SITAW_AR_THRESHOLD.
   */
  triggerTestValue(&sitAwARAccZ, fabs(accZ - 1));
}

/**
 * Check if an At Rest situation has been detected.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwARDetected(void)
{
  return sitAwARAccZ.released;
}
#endif

#ifdef SITAW_TU_ENABLED
/**
 * Initialize the Tumbled detection.
 *
 * See the sitAwTuTest() function for details.
 */
void sitAwTuInit(void)
{
  triggerInit(&sitAwTuAcc, triggerFuncIsLE, SITAW_TU_ACC_THRESHOLD, SITAW_TU_ACC_TRIGGER_COUNT);
  triggerActivate(&sitAwTuAcc, true);
}

/**
 * Test values for a Tumbled situation.
 *
 * A tumbled situation is considered identified when accelerometer reports
 * a negative value.
 *
 * Once a tumbled situation is identified, this can be used for instance to
 * cut the thrust to the motors, avoiding the crazyflie from running
 * propellers at significant thrust when accidentally crashing into walls
 * or the ground.

 * @param The current accelerometer reading in z direction
 */
void sitAwTuTest(float accz)
{
  triggerTestValue(&sitAwTuAcc, accz);
}

/**
 * Check if a Tumbled situation has been detected.
 *
 * @return True if the situation has been detected, otherwise false.
 */
bool sitAwTuDetected(void)
{
  return sitAwTuAcc.released;
}
#endif

/**
 * Initialize the situation awareness subsystem.
 */
void sitAwInit(void)
{
#ifdef SITAW_FF_ENABLED
  sitAwFFInit();
#endif
#ifdef SITAW_AR_ENABLED
  sitAwARInit();
#endif
#ifdef SITAW_TU_ENABLED
  sitAwTuInit();
#endif
}
