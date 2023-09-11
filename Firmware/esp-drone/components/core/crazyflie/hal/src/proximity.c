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
 * proximity.c - Implementation of hardware abstraction layer for proximity sensors
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "proximity.h"
#include "maxsonar.h"
#include "system.h"
#include "param.h"
#include "log.h"

#include "stm32_legacy.h"
#include "static_mem.h"

/* Flag indicating if the proximityInit() function has been called or not. */
static bool isInit = false;

/* Internal values exported by functions below. */
static uint32_t proximityDistance       = 0; /* The distance measured in millimeters for the latest sample. */
static uint32_t proximityDistanceAvg    = 0; /* Average distance in millimeters, initialized to zero. */
static uint32_t proximityDistanceMedian = 0; /* Median distance in millimeters, initialized to zero. */
static uint32_t proximityAccuracy       = 0; /* The accuracy as reported by the sensor driver for the latest sample. */

/* The most recent samples in chronological order. Must be initialized before use. */
static uint32_t proximitySWin[PROXIMITY_SWIN_SIZE];

#if defined(PROXIMITY_ENABLED)

#if defined(PROXIMITY_LOG_ENABLED)
/* Define a log group. */
LOG_GROUP_START(proximity)
LOG_ADD(LOG_UINT32, distance, &proximityDistance)
LOG_ADD(LOG_UINT32, distanceAvg, &proximityDistanceAvg)
LOG_ADD(LOG_UINT32, distanceMed, &proximityDistanceMedian)
LOG_ADD(LOG_UINT32, accuracy, &proximityAccuracy)
LOG_GROUP_STOP(proximity)
#endif

STATIC_MEM_TASK_ALLOC(proximityTask, PROXIMITY_TASK_STACKSIZE);

/**
 * This function returns the median value of an array.
 *
 * Internal sorting function by Bill Gentles Nov. 12 2010, seen
 * on http://forum.arduino.cc/index.php?topic=20920.0
 *
 * @param proximitySWin Array of chronologically sequenced samples.
 *
 * @return Median value from the array.
 */
static uint32_t proximitySWinMedian(uint32_t *proximitySWin)
{
  /* The most recent samples, sorted in increasing sample value order. Must be initialized before use. */
  uint32_t proximitySorted[PROXIMITY_SWIN_SIZE];

  /* Create a copy of the chronologically sequenced buffer. */
  memcpy(proximitySorted, proximitySWin, sizeof(uint32_t)*PROXIMITY_SWIN_SIZE);

  /* Now sort this copy. */
  uint8_t n;
  for (n = 1; n < PROXIMITY_SWIN_SIZE; ++n) {
    uint32_t valn = proximitySorted[n];
    int8_t m; /* May reach value of -1 */
    for (m = n - 1; (m >= 0) && (valn < proximitySorted[m]); m--)
    {
      proximitySorted[m + 1] = proximitySorted[m];
    }
    proximitySorted[m + 1] = valn;
  }

  /* Return the median value of the samples. */
  return proximitySorted[PROXIMITY_SWIN_SIZE / 2];
}

/**
 * This function adds a distance measurement to the sliding window, discarding the oldest sample.
 * After having added the new sample, a new average value of the samples is calculated and returned.
 *
 * @param distance The new sample to add to the sliding window.
 *
 * @return The new average value of the samples in the sliding window (after adding the new sample).
 */
static uint32_t proximitySWinAdd(uint32_t distance)
{
  /* Discard oldest sample, move remaining samples one slot to the left. */
  memmove(&proximitySWin[0], &proximitySWin[1], (PROXIMITY_SWIN_SIZE - 1) * sizeof(uint32_t));

  /* Add the new sample in the last (right-most) slot. */
  proximitySWin[PROXIMITY_SWIN_SIZE - 1] = distance;

  /**
   * Calculate the new average distance. Sum all the samples into a uint64_t,
   * so that we only do a single division at the end.
   */
  uint64_t proximityNewAvg = 0;
  uint8_t n;
  for (n = 0; n < PROXIMITY_SWIN_SIZE; n++) {
    proximityNewAvg += proximitySWin[n];
  }
  proximityNewAvg = proximityNewAvg / PROXIMITY_SWIN_SIZE;

  return (uint32_t)proximityNewAvg;
}

/**
 * Proximity task running at PROXIMITY_TASK_FREQ Hz.
 *
 * @param param Currently unused.
 */
static void proximityTask(void* param)
{
  uint32_t lastWakeTime;

#ifdef configUSE_APPLICATION_TASK_TAG
	#if configUSE_APPLICATION_TASK_TAG == 1
  vTaskSetApplicationTaskTag(0, (void*)TASK_PROXIMITY_ID_NBR);
  #endif
#endif

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(PROXIMITY_TASK_FREQ));

#if defined(MAXSONAR_ENABLED)
    /* Read the MaxBotix sensor. */
    proximityDistance = maxSonarReadDistance(MAXSONAR_MB1040_AN, &proximityAccuracy);
#endif

    /* Get the latest average value calculated. */
    proximityDistanceAvg = proximitySWinAdd(proximityDistance);

    /* Get the latest median value calculated. */
    proximityDistanceMedian = proximitySWinMedian(proximitySWin);
  }
}
#endif

/**
 * Initialization of the proximity task.
 */
void proximityInit(void)
{
  if(isInit)
    return;

  /* Initialise the sliding window to zero. */
  memset(&proximitySWin, 0, sizeof(uint32_t)*PROXIMITY_SWIN_SIZE);

#if defined(PROXIMITY_ENABLED)
  /* Only start the task if the proximity subsystem is enabled in conf.h */
  STATIC_MEM_TASK_CREATE(proximityTask, proximityTask, PROXIMITY_TASK_NAME, NULL, PROXIMITY_TASK_PRI);
#endif

  isInit = true;
}

/**
 * Function returning the last proximity measurement.
 *
 * @return The last proximity measurement made.
 */
uint32_t proximityGetDistance(void)
{
  return proximityDistance;
}

/**
 * Function returning the result of the last, average proximity calculation.
 * The calculation is a simple average of the last PROXIMITY_SWIN_SIZE samples.
 *
 * @return The result from the last, average proximity calculation.
 */
uint32_t proximityGetDistanceAvg(void)
{
  return proximityDistanceAvg;
}

/**
 * Function returning the result of the last, median proximity calculation.
 * The calculation is the median of the last PROXIMITY_SWIN_SIZE samples.
 *
 * @return The result from the last, median proximity calculation.
 */
uint32_t proximityGetDistanceMedian(void)
{
  return proximityDistanceMedian;
}

/**
 * Function returning the accuracy of the last proximity measurement.
 *
 * @return The accuracy of the last proximity measurement made.
 */
uint32_t proximityGetAccuracy(void)
{
  return proximityAccuracy;
}
