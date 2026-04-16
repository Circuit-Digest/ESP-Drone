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
 * sitaw.h - Interface to situation awareness.
 */

#ifndef __SITAW_H__
#define __SITAW_H__

#include <stdint.h>
#include <stdbool.h>

#include "stabilizer_types.h"

void sitAwInit(void);
void sitAwUpdateSetpoint(setpoint_t *setpoint, const sensorData_t *sensorData,
                                               const state_t *state);
/* Enable the situation awareness framework. */
#define SITAW_ENABLED
/* Enable the different functions of the situation awareness framework. */
//#define SITAW_FF_ENABLED           /* Uncomment to enable */
//#define SITAW_AR_ENABLED           /* Uncomment to enable */
#define SITAW_TU_ENABLED           /* Uncomment to enable */

/* Configuration options for the 'Free Fall' detection. */
#define SITAW_FF_THRESHOLD 0.1     /* The default tolerance for AccWZ deviations from -1, indicating Free Fall. */
#define SITAW_FF_TRIGGER_COUNT 15  /* The number of consecutive tests for Free Fall to be detected. Configured for 250Hz testing. */

/* Configuration options for the 'At Rest' detection. */
#define SITAW_AR_THRESHOLD 0.05    /* The default tolerance for AccZ deviations from 1 and AccX, AccY deviations from 0, indicating At Rest. */
#define SITAW_AR_TRIGGER_COUNT 500 /* The number of consecutive tests for At Rest to be detected. Configured for 250Hz testing. */

/* Configuration options for the 'Tumbled' detection. */
#define SITAW_TU_ACC_THRESHOLD (-0.5f)     /* The maximum acc.z value indicating a Tumbled situation. */
#define SITAW_TU_ACC_TRIGGER_COUNT 30      /* The number of consecutive tests for Tumbled to be detected. Configured for 250Hz testing. */
#define SITAW_TU_IN_FLIGHT_THRESHOLD 1000  /* Minimum summed motor PWM that means we are flying */

/* LOG configurations. Enable these to be able to log detection in the cfclient. */
#define SITAW_LOG_ENABLED            /* Uncomment to enable LOG framework. */
//#define SITAW_FF_LOG_ENABLED       /* Uncomment to enable LOG framework for the Free Fall detection trigger object. */
//#define SITAW_AR_LOG_ENABLED       /* Uncomment to enable LOG framework for the At Rest detection trigger object. */
//#define SITAW_TU_LOG_ENABLED       /* Uncomment to enable LOG framework for the Tumbled detection trigger object. */
#define SITAW_LOG_ALL_DETECT_ENABLED /* Uncomment to enable LOG framework for all 'Detected' flags. */

/* PARAM configurations. Enable these to be able to tweak detection configurations from the cfclient. */
//#define SITAW_PARAM_ENABLED        /* Uncomment to enable PARAM framework. */
//#define SITAW_FF_PARAM_ENABLED     /* Uncomment to enable PARAM framework for the Free Fall detection. */
//#define SITAW_AR_PARAM_ENABLED     /* Uncomment to enable PARAM framework for the At Rest detection. */
//#define SITAW_TU_PARAM_ENABLED     /* Uncomment to enable PARAM framework for the Tumbled detection. */

#ifdef SITAW_FF_ENABLED
bool sitAwFFDetected(void);
#endif
#ifdef SITAW_AR_ENABLED
bool sitAwARDetected(void);
#endif
#ifdef SITAW_TU_ENABLED
bool sitAwTuDetected(void);
#endif

#endif
