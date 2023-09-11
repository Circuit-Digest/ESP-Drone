/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * outlierFilter.c: Outlier rejection filter for the LPS system
 */

#include <math.h>
#include "outlierFilter.h"
#include "stabilizer_types.h"
#include "log.h"
#include "debug_cf.h"

#define BUCKET_ACCEPTANCE_LEVEL 3
#define MAX_BUCKET_FILL 10
#define FILTER_CLOSE_DELAY_COUNT 30

static float acceptanceLevel = 0.0;
static float errorDistance;
static int filterCloseDelayCounter = 0;
static int previousFilterIndex = 0;

typedef struct {
  float acceptanceLevel;
  int bucket;
} filterLevel_t;

#define FILTER_LEVELS 5
#define FILTER_NONE FILTER_LEVELS
filterLevel_t filterLevels[FILTER_LEVELS] = {
  {.acceptanceLevel = 0.4},
  {.acceptanceLevel = 0.8},
  {.acceptanceLevel = 1.2},
  {.acceptanceLevel = 1.6},
  {.acceptanceLevel = 2.0},
};


static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(const tdoaMeasurement_t* tdoa);
static float distanceSq(const point_t* a, const point_t* b);
static float sq(float a) {return a * a;}
static void addToBucket(filterLevel_t* filter);
static void removeFromBucket(filterLevel_t* filter);
static int updateBuckets(float errorDistance);



bool outlierFilterValidateTdoaSimple(const tdoaMeasurement_t* tdoa) {
  return isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa);
}

bool outlierFilterValidateTdoaSteps(const tdoaMeasurement_t* tdoa, const float error, const vector_t* jacobian, const point_t* estPos) {
  bool sampleIsGood = false;

  if (isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa)) {
    float errorBaseDistance = sqrtf(powf(jacobian->x, 2) + powf(jacobian->y, 2) + powf(jacobian->z, 2));
    errorDistance = fabsf(error / errorBaseDistance);

    int filterIndex = updateBuckets(errorDistance);

    if (filterIndex > previousFilterIndex) {
      filterCloseDelayCounter = FILTER_CLOSE_DELAY_COUNT;
    } else if (filterIndex < previousFilterIndex) {
      if (filterCloseDelayCounter > 0) {
        filterCloseDelayCounter--;
        filterIndex = previousFilterIndex;
      }
    }
    previousFilterIndex = filterIndex;

    if (filterIndex == FILTER_NONE) {
      // Lost tracking, open up to let the kalman filter converge
      acceptanceLevel = 100.0;
      sampleIsGood = true;
    } else {
      acceptanceLevel = filterLevels[filterIndex].acceptanceLevel;
      if (errorDistance < acceptanceLevel) {
        sampleIsGood = true;
      }
    }
  }

  return sampleIsGood;
}


#define LH_TICKS_PER_FRAME (1000 / 120)
static const int32_t lhMinWindowTime = -2 * LH_TICKS_PER_FRAME;
static const int32_t lhMaxWindowTime = 5 * LH_TICKS_PER_FRAME;
static const int32_t lhBadSampleWindowChange = -LH_TICKS_PER_FRAME;
static const int32_t lhGoodSampleWindowChange = LH_TICKS_PER_FRAME / 2;
static const float lhMaxError = 0.05f;

void outlierFilterReset(OutlierFilterLhState_t* this, const uint32_t now) {
  this->openingTime = now;
  this->openingWindow = lhMinWindowTime;
}


bool outlierFilterValidateLighthouseSweep(OutlierFilterLhState_t* this, const float distanceToBs, const float angleError, const uint32_t now) {
  // float error = distanceToBs * tan(angleError);
  // We use an approximattion
  float error = distanceToBs * angleError;

  bool isGoodSample = (fabsf(error) < lhMaxError);
  if (isGoodSample) {
    this->openingWindow += lhGoodSampleWindowChange;
    if (this->openingWindow > lhMaxWindowTime) {
      this->openingWindow = lhMaxWindowTime;
    }
  } else {
    this->openingWindow += lhBadSampleWindowChange;
    if (this->openingWindow < lhMinWindowTime) {
      this->openingWindow = lhMinWindowTime;
    }
  }

  bool result = true;
  bool isFilterClosed = (now < this->openingTime);
  if (isFilterClosed) {
    result = isGoodSample;
  }

  this->openingTime = now + this->openingWindow;

  return result;
}


static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(const tdoaMeasurement_t* tdoa) {
  float anchorDistanceSq = distanceSq(&tdoa->anchorPosition[0], &tdoa->anchorPosition[1]);
  float distanceDiffSq = sq(tdoa->distanceDiff);
  return (distanceDiffSq < anchorDistanceSq);
}

static float distanceSq(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}


static void addToBucket(filterLevel_t* filter) {
  if (filter->bucket < MAX_BUCKET_FILL) {
    filter->bucket++;
  }
}

static void removeFromBucket(filterLevel_t* filter) {
  if (filter->bucket > 0) {
    filter->bucket--;
  }
}

static int updateBuckets(float errorDistance) {
  int filterIndex = FILTER_NONE;

  for (int i = FILTER_LEVELS - 1; i >= 0; i--) {
    filterLevel_t* filter = &filterLevels[i];

    if (errorDistance < filter->acceptanceLevel) {
      removeFromBucket(filter);
    } else {
      addToBucket(filter);
    }

    if (filter->bucket < BUCKET_ACCEPTANCE_LEVEL) {
      filterIndex = i;
    }
  }

  return filterIndex;
}

LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, bucket0, &filterLevels[0].bucket)
  LOG_ADD(LOG_INT32, bucket1, &filterLevels[1].bucket)
  LOG_ADD(LOG_INT32, bucket2, &filterLevels[2].bucket)
  LOG_ADD(LOG_INT32, bucket3, &filterLevels[3].bucket)
  LOG_ADD(LOG_INT32, bucket4, &filterLevels[4].bucket)
  LOG_ADD(LOG_FLOAT, accLev, &acceptanceLevel)
  LOG_ADD(LOG_FLOAT, errD, &errorDistance)
LOG_GROUP_STOP(outlierf)
