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
 * estimator.h - State estimator interface
 */
#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__

#include "stabilizer_types.h"

typedef enum {
  anyEstimator = 0,
  complementaryEstimator,
  kalmanEstimator,
  StateEstimatorTypeCount,
} StateEstimatorType;

bool registerRequiredEstimator(StateEstimatorType estimator);
StateEstimatorType deckGetRequiredEstimator();
void stateEstimatorInit(StateEstimatorType estimator);
bool stateEstimatorTest(void);
void stateEstimatorSwitchTo(StateEstimatorType estimator);
void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
StateEstimatorType getStateEstimator(void);
const char* stateEstimatorGetName();

// Support to incorporate additional sensors into the state estimate via the following functions:
bool estimatorEnqueueTDOA(const tdoaMeasurement_t *uwb);
bool estimatorEnqueuePosition(const positionMeasurement_t *pos);
bool estimatorEnqueuePose(const poseMeasurement_t *pose);
bool estimatorEnqueueDistance(const distanceMeasurement_t *dist);
bool estimatorEnqueueTOF(const tofMeasurement_t *tof);
bool estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height);
bool estimatorEnqueueFlow(const flowMeasurement_t *flow);
bool estimatorEnqueueYawError(const yawErrorMeasurement_t *error);
//bool estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *angles);

#endif //__ESTIMATOR_H__
