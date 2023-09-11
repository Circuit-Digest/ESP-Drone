/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 * lighthouse_position_est.c - position estimaton for the lighthouse system
 */

#include "stabilizer_types.h"
#include "estimator.h"
#include "estimator_kalman.h"
#include "math.h"
#include "cf_math.h"

#include "log.h"
#include "param.h"
#include "statsCnt.h"
#include "mem.h"

#include "lighthouse_position_est.h"
#include "lighthouse_geometry.h"
#include "lighthouse_state.h"

// lighthouseBaseStationsGeometry has been moved to lighthouse_core.c

#define ONE_SECOND 1000
#define HALF_SECOND 500
static STATS_CNT_RATE_DEFINE(positionRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(estBs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(estBs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsEstRates[PULSE_PROCESSOR_N_BASE_STATIONS] = {&estBs0Rate, &estBs1Rate};

// The light planes in LH2 are tilted +- 30 degrees
static const float t30 = M_PI / 6;

static void lighthousePositionGeometryDataUpdated(const int baseStation);
static void preProcessGeometryData(mat3d bsRot, mat3d bsRotInverted, mat3d lh1Rotor2Rot, mat3d lh1Rotor2RotInverted);

// Geometry memory handling for the memory module
static const uint32_t calibStartAddr = 0x1000;
static const uint32_t pageSize = 0x100;
static uint32_t handleMemGetSize(void) { return calibStartAddr + sizeof(lighthouseCoreState.bsCalibration); }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_LH,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

void lighthousePositionEstInit() {
  for (int i = 0; i < PULSE_PROCESSOR_N_BASE_STATIONS; i++) {
    lighthousePositionGeometryDataUpdated(i);
  }
  memoryRegisterHandler(&memDef);
}

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  if (memAddr < calibStartAddr) {
    uint32_t index = memAddr / pageSize;
    uint32_t inPageAddr = memAddr % pageSize;
    if (index < PULSE_PROCESSOR_N_BASE_STATIONS) {
      if (inPageAddr + readLen <= sizeof(baseStationGeometry_t)) {
        uint8_t* start = (uint8_t*)&lighthouseCoreState.bsGeometry[index];
        memcpy(buffer, start + inPageAddr, readLen);

        result = true;
      }
    }
  } else {
    uint32_t calibOffsetAddr = memAddr - calibStartAddr;
    uint32_t index = calibOffsetAddr / pageSize;
    uint32_t inPageAddr = calibOffsetAddr % pageSize;
    if (index < PULSE_PROCESSOR_N_BASE_STATIONS) {
      if (inPageAddr + readLen <= sizeof(lighthouseCalibration_t)) {
        uint8_t* start = (uint8_t*)&lighthouseCoreState.bsCalibration[index];
        memcpy(buffer, start + inPageAddr, readLen);

        result = true;
      }
    }
  }

  return result;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  if (memAddr < calibStartAddr) {
    uint32_t index = memAddr / pageSize;
    uint32_t inPageAddr = memAddr % pageSize;
    if (index < PULSE_PROCESSOR_N_BASE_STATIONS) {
      if (inPageAddr + writeLen <= sizeof(baseStationGeometry_t)) {
        // Mark the geometry as invalid since this write probably only will update part of it
        // If this is the last write in this block, the valid flag will be part of the data and set appropriately
        // This is based on the assumption that the writes are done in oder with increasing addresses
        lighthouseCoreState.bsGeometry[index].valid = false;

        uint8_t* start = (uint8_t*)&lighthouseCoreState.bsGeometry[index];
        memcpy(start + inPageAddr, buffer, writeLen);

        lighthousePositionGeometryDataUpdated(index);

        result = true;
      }
    }
  } else {
    uint32_t calibOffsetAddr = memAddr - calibStartAddr;
    uint32_t index = calibOffsetAddr / pageSize;
    uint32_t inPageAddr = calibOffsetAddr % pageSize;
    if (index < PULSE_PROCESSOR_N_BASE_STATIONS) {
      if (inPageAddr + writeLen <= sizeof(lighthouseCalibration_t)) {
        uint8_t* start = (uint8_t*)&lighthouseCoreState.bsCalibration[index];
        memcpy(start + inPageAddr, buffer, writeLen);

        result = true;
      }
    }
  }

  return result;
}

static void lighthousePositionGeometryDataUpdated(const int baseStation) {
  if (lighthouseCoreState.bsGeometry[baseStation].valid) {
    baseStationGeometryCache_t* cache = &lighthouseCoreState.bsGeoCache[baseStation];
    preProcessGeometryData(lighthouseCoreState.bsGeometry[baseStation].mat, cache->baseStationInvertedRotationMatrixes, cache->lh1Rotor2RotationMatrixes, cache->lh1Rotor2InvertedRotationMatrixes);
  }
}

void lighthousePositionSetGeometryData(const uint8_t baseStation, const baseStationGeometry_t* geometry) {
  if (baseStation < PULSE_PROCESSOR_N_BASE_STATIONS) {
    lighthouseCoreState.bsGeometry[baseStation] = *geometry;
    lighthousePositionGeometryDataUpdated(baseStation);
  }
}

static void preProcessGeometryData(mat3d bsRot, mat3d bsRotInverted, mat3d lh1Rotor2Rot, mat3d lh1Rotor2RotInverted) {
  // For a rotation matrix inverse and transpose is equal. Use transpose instead
  arm_matrix_instance_f32 bsRot_ = {3, 3, (float32_t *)bsRot};
  arm_matrix_instance_f32 bsRotInverted_ = {3, 3, (float32_t *)bsRotInverted};
  mat_trans(&bsRot_, &bsRotInverted_);

  // In a LH1 system, the axis of rotation of the second rotor is perpendicular to the first rotor
  mat3d secondRotorInvertedR = {
    {1, 0, 0},
    {0, 0, -1},
    {0, 1, 0}
  };
  arm_matrix_instance_f32 secondRotorInvertedR_ = {3, 3, (float32_t *)secondRotorInvertedR};
  arm_matrix_instance_f32 lh1Rotor2Rot_ = {3, 3, (float32_t *)lh1Rotor2Rot};
  mat_mult(&bsRot_, &secondRotorInvertedR_, &lh1Rotor2Rot_);

  arm_matrix_instance_f32 lh1Rotor2RotInverted_ = {3, 3, (float32_t *)lh1Rotor2RotInverted};
  mat_trans(&lh1Rotor2Rot_, &lh1Rotor2RotInverted_);
}


// Sensor positions on the deck
#define SENSOR_POS_W (0.015f / 2.0f)
#define SENSOR_POS_L (0.030f / 2.0f)
static vec3d sensorDeckPositions[4] = {
    {-SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {-SENSOR_POS_L, -SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, -SENSOR_POS_W, 0.0},
};


static positionMeasurement_t ext_pos;
static float sweepStd = 0.0004;
static float sweepStdLh2 = 0.001;

static vec3d position;
static float deltaLog;

static void estimatePositionCrossingBeams(const pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation) {
  memset(&ext_pos, 0, sizeof(ext_pos));
  int sensorsUsed = 0;
  float delta;

  // Average over all sensors with valid data
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      pulseProcessorBaseStationMeasuremnt_t* bs0Measurement = &angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[0];
      pulseProcessorBaseStationMeasuremnt_t* bs1Measurement = &angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[1];

      if (bs0Measurement->validCount == PULSE_PROCESSOR_N_SWEEPS && bs1Measurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
        lighthouseGeometryGetPositionFromRayIntersection(state->bsGeometry, bs0Measurement->correctedAngles, bs1Measurement->correctedAngles, position, &delta);

        deltaLog = delta;

        ext_pos.x += position[0];
        ext_pos.y += position[1];
        ext_pos.z += position[2];
        sensorsUsed++;

        STATS_CNT_RATE_EVENT(&positionRate);
      }
  }

  ext_pos.x /= sensorsUsed;
  ext_pos.y /= sensorsUsed;
  ext_pos.z /= sensorsUsed;

  // Make sure we feed sane data into the estimator
  if (isfinite(ext_pos.pos[0]) && isfinite(ext_pos.pos[1]) && isfinite(ext_pos.pos[2])) {
    ext_pos.stdDev = 0.01;
    estimatorEnqueuePosition(&ext_pos);
  }
}

static void estimatePositionSweepsLh1(const pulseProcessor_t* appState, pulseProcessorResult_t* angles, int baseStation) {
  const lighthouseCalibration_t* bsCalib = &appState->bsCalibration[baseStation];
  sweepAngleMeasurement_t sweepInfo;
  sweepInfo.stdDev = sweepStd;
  sweepInfo.rotorPos = &appState->bsGeometry[baseStation].origin;
  sweepInfo.t = 0;
  sweepInfo.calibrationMeasurementModel = lighthouseCalibrationMeasurementModelLh1;

  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[baseStation];
    if (bsMeasurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
      sweepInfo.sensorPos = &sensorDeckPositions[sensor];

      sweepInfo.measuredSweepAngle = bsMeasurement->angles[0];
      if (sweepInfo.measuredSweepAngle != 0) {
        sweepInfo.rotorRot = &appState->bsGeometry[baseStation].mat;
        sweepInfo.rotorRotInv = &appState->bsGeoCache[baseStation].baseStationInvertedRotationMatrixes;
        sweepInfo.calib = &bsCalib->sweep[0];

        estimatorEnqueueSweepAngles(&sweepInfo);
        STATS_CNT_RATE_EVENT(bsEstRates[baseStation]);
        STATS_CNT_RATE_EVENT(&positionRate);
      }

      sweepInfo.measuredSweepAngle = bsMeasurement->angles[1];
      if (sweepInfo.measuredSweepAngle != 0) {
        sweepInfo.rotorRot = &appState->bsGeoCache[baseStation].lh1Rotor2RotationMatrixes;
        sweepInfo.rotorRotInv = &appState->bsGeoCache[baseStation].lh1Rotor2InvertedRotationMatrixes;
        sweepInfo.calib = &bsCalib->sweep[1];

        estimatorEnqueueSweepAngles(&sweepInfo);
        STATS_CNT_RATE_EVENT(bsEstRates[baseStation]);
        STATS_CNT_RATE_EVENT(&positionRate);
      }
    }
  }
}

static void estimatePositionSweepsLh2(const pulseProcessor_t* appState, pulseProcessorResult_t* angles, int baseStation) {
  const lighthouseCalibration_t* bsCalib = &appState->bsCalibration[baseStation];
  sweepAngleMeasurement_t sweepInfo;
  sweepInfo.stdDev = sweepStdLh2;
  sweepInfo.rotorPos = &appState->bsGeometry[baseStation].origin;
  sweepInfo.rotorRot = &appState->bsGeometry[baseStation].mat;
  sweepInfo.rotorRotInv = &appState->bsGeoCache[baseStation].baseStationInvertedRotationMatrixes;
  sweepInfo.calibrationMeasurementModel = lighthouseCalibrationMeasurementModelLh2;

  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurementsLh2[sensor].baseStatonMeasurements[baseStation];
    if (bsMeasurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
      sweepInfo.sensorPos = &sensorDeckPositions[sensor];

      sweepInfo.measuredSweepAngle = bsMeasurement->angles[0];
      if (sweepInfo.measuredSweepAngle != 0) {
        sweepInfo.t = -t30;
        sweepInfo.calib = &bsCalib->sweep[0];
        estimatorEnqueueSweepAngles(&sweepInfo);
        STATS_CNT_RATE_EVENT(bsEstRates[baseStation]);
        STATS_CNT_RATE_EVENT(&positionRate);
      }

      sweepInfo.measuredSweepAngle = bsMeasurement->angles[1];
      if (sweepInfo.measuredSweepAngle != 0) {
        sweepInfo.t = t30;
        sweepInfo.calib = &bsCalib->sweep[1];
        estimatorEnqueueSweepAngles(&sweepInfo);
        STATS_CNT_RATE_EVENT(bsEstRates[baseStation]);
        STATS_CNT_RATE_EVENT(&positionRate);
      }
    }
  }
}

static void estimatePositionSweeps(const pulseProcessor_t* appState, pulseProcessorResult_t* angles, int baseStation) {
  switch(angles->measurementType) {
    case lighthouseBsTypeV1:
      estimatePositionSweepsLh1(appState, angles, baseStation);
      break;
    case lighthouseBsTypeV2:
      estimatePositionSweepsLh2(appState, angles, baseStation);
      break;
    default:
      // Do nothing
      break;
  }
}

static bool estimateYawDeltaOneBaseStation(const int bs, const pulseProcessorResult_t* angles, const baseStationGeometry_t baseStationGeometries[], const float cfPos[3], const float n[3], const arm_matrix_instance_f32 *RR, float *yawDelta) {
  const baseStationGeometry_t* baseStationGeometry = &baseStationGeometries[bs];

  vec3d baseStationPos;
  lighthouseGeometryGetBaseStationPosition(baseStationGeometry, baseStationPos);

  vec3d rays[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    const pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[bs];
    lighthouseGeometryGetRay(baseStationGeometry, bsMeasurement->correctedAngles[0], bsMeasurement->correctedAngles[1], rays[sensor]);
  }

  // Intersection points of rays and the deck
  vec3d intersectionPoints[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    bool exists = lighthouseGeometryIntersectionPlaneVector(baseStationPos, rays[sensor], cfPos, n, intersectionPoints[sensor]);
    if (! exists) {
      return false;
    }
  }

  // Calculate positions of sensors. Rotate relative postiions using the rotation matrix and add current position
  vec3d sensorPoints[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    lighthouseGeometryGetSensorPosition(cfPos, RR, sensorDeckPositions[sensor], sensorPoints[sensor]);
  }

  // Calculate diagonals (sensors 0 - 3 and 1 - 2) for intersection and sensor points
  vec3d ipv1 = {intersectionPoints[3][0] - intersectionPoints[0][0], intersectionPoints[3][1] - intersectionPoints[0][1], intersectionPoints[3][2] - intersectionPoints[0][2]};
  vec3d ipv2 = {intersectionPoints[2][0] - intersectionPoints[1][0], intersectionPoints[2][1] - intersectionPoints[1][1], intersectionPoints[2][2] - intersectionPoints[1][2]};
  vec3d spv1 = {sensorPoints[3][0] - sensorPoints[0][0], sensorPoints[3][1] - sensorPoints[0][1], sensorPoints[3][2] - sensorPoints[0][2]};
  vec3d spv2 = {sensorPoints[2][0] - sensorPoints[1][0], sensorPoints[2][1] - sensorPoints[1][1], sensorPoints[2][2] - sensorPoints[1][2]};

  // Calculate yaw delta for the two diagonals and average
  float yawDelta1, yawDelta2;
  if (lighthouseGeometryYawDelta(ipv1, spv1, n, &yawDelta1) && lighthouseGeometryYawDelta(ipv2, spv2, n, &yawDelta2)) {
    *yawDelta = (yawDelta1 + yawDelta2) / 2.0f;
    return true;
   } else {
    *yawDelta = 0.0f;
    return false;
  }
}

static void estimateYaw(const pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation) {
  // TODO Most of these calculations should be moved into the estimator instead. It is a
  // bit dirty to get the state from the kalman filer here and calculate the yaw error outside
  // the estimator, but it will do for now.

  // Get data from the current estimated state
  point_t cfPosP;
  estimatorKalmanGetEstimatedPos(&cfPosP);
  vec3d cfPos = {cfPosP.x, cfPosP.y, cfPosP.z};

  // Rotation matrix
  float R[3][3];
  estimatorKalmanGetEstimatedRot((float*)R);
  arm_matrix_instance_f32 RR = {3, 3, (float*)R};

  // Normal to the deck: (0, 0, 1), rotated using the rotation matrix
  const vec3d n = {R[0][2], R[1][2], R[2][2]};

  // Calculate yaw delta using only one base station for now
  float yawDelta;
  if (estimateYawDeltaOneBaseStation(baseStation, angles, state->bsGeometry, cfPos, n, &RR, &yawDelta)) {
    yawErrorMeasurement_t yawDeltaMeasurement = {.yawError = yawDelta, .stdDev = 0.01};
    estimatorEnqueueYawError(&yawDeltaMeasurement);
  }
}

void lighthousePositionEstimatePoseCrossingBeams(const pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation) {
  if (state->bsGeometry[0].valid && state->bsGeometry[1].valid) {
    estimatePositionCrossingBeams(state, angles, baseStation);
    estimateYaw(state, angles, baseStation);
  }
}

void lighthousePositionEstimatePoseSweeps(const pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation) {
  if (state->bsGeometry[baseStation].valid) {
    estimatePositionSweeps(state, angles, baseStation);
    estimateYaw(state, angles, baseStation);
  }
}


LOG_GROUP_START(lighthouse)
STATS_CNT_RATE_LOG_ADD(posRt, &positionRate)
STATS_CNT_RATE_LOG_ADD(estBs0Rt, &estBs0Rate)
STATS_CNT_RATE_LOG_ADD(estBs1Rt, &estBs1Rate)

LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])

LOG_ADD(LOG_FLOAT, delta, &deltaLog)
LOG_GROUP_STOP(lighthouse)

PARAM_GROUP_START(lighthouse)
PARAM_ADD(PARAM_FLOAT, sweepStd, &sweepStd)
PARAM_ADD(PARAM_FLOAT, sweepStd2, &sweepStdLh2)
PARAM_GROUP_STOP(lighthouse)
