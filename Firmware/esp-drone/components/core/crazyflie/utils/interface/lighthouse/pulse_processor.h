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
 * pulse_processor.h - pulse decoding for lighthouse V1 base stations
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "ootx_decoder.h"
#include "lighthouse_calibration.h"
#include "lighthouse_geometry.h"

#define PULSE_PROCESSOR_N_SWEEPS 2
#define PULSE_PROCESSOR_N_BASE_STATIONS 2
#define PULSE_PROCESSOR_N_SENSORS 4
#define PULSE_PROCRSSOR_N_CONCURRENT_BLOCKS 2
#define PULSE_PROCESSOR_N_WORKSPACE (PULSE_PROCESSOR_N_SENSORS * PULSE_PROCRSSOR_N_CONCURRENT_BLOCKS)

#define PULSE_PROCESSOR_HISTORY_LENGTH 8
#define PULSE_PROCESSOR_TIMESTAMP_BITWIDTH 24
#define PULSE_PROCESSOR_TIMESTAMP_MAX ((1 << PULSE_PROCESSOR_TIMESTAMP_BITWIDTH) - 1)
#define PULSE_PROCESSOR_TIMESTAMP_BITMASK PULSE_PROCESSOR_TIMESTAMP_MAX

// Utility functions and macros

/**
 * @brief Difference of two timestamps, truncated to time stamp bit width (PULSE_PROCESSOR_TIMESTAMP_BITWIDTH)
 *
 * @param x A timestamp
 * @param y A teimstamp
 * @return x - y, truncated
 */
inline static uint32_t TS_DIFF(const uint32_t x, const uint32_t y) {
  return (x - y) & PULSE_PROCESSOR_TIMESTAMP_BITMASK;
}

/**
 * @brief Check if abs(a - b) > limit. Works for timestampa where the bitwidth is PULSE_PROCESSOR_TIMESTAMP_BITWIDTH
 *
 * @param a A timestamp
 * @param b A timestamp
 * @param limit The minimum difference between a nd b
 * @return true if abs(a - b) > limit
 */
inline static uint32_t TS_ABS_DIFF_LARGER_THAN(const uint32_t a, const uint32_t b, const uint32_t limit) {
    return TS_DIFF(a + limit, b) > (limit * 2);
}


/**
 * @brief Data for one pulse, detected by one sensor and decoded by the FPGA on the dack.
 * Used both for lighthouse V1 and V2.
 *
 */
typedef struct {
  uint8_t sensor;
  uint32_t timestamp;

  // V1 base station data --------
  uint16_t width;

  // V2 base station data --------
  uint32_t beamData;
  uint32_t offset;
  // Channel is zero indexed (0-15) here, while it is one indexed in the base station config (1 - 16)
  uint8_t channel; // Valid if channelFound is true
  uint8_t slowbit; // Valid if channelFound is true
  bool channelFound;
} pulseProcessorFrame_t;

typedef enum {
    lighthouseBsTypeUnknown = 0,
    lighthouseBsTypeV1 = 1,
    lighthouseBsTypeV2 = 2,
} lighthouseBaseStationType_t;

enum pulseClass_e {unknown, sync0, sync1, sweep};

typedef struct {
  uint32_t timestamp;
  int width;
} pulseProcessorPulse_t;

typedef enum {
  sweepIdFirst = 0,  // The X sweep in LH 1
  sweepIdSecond = 1  // The Y sweep in LH 1
} SweepId_t;

typedef enum {
  sweepStorageStateWaiting = 0,
  sweepStorageStateValid,
  sweepStorageStateError,
} SweepStorageState_t;

/**
 * @brief Holds data for V1 base station decoding
 *
 */
typedef struct {
  bool synchronized;    // At true if we are currently syncthonized
  int basestationsSynchronizedCount;

  // Synchronization state
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_N_SENSORS][PULSE_PROCESSOR_HISTORY_LENGTH];
  int pulseHistoryIdx[PULSE_PROCESSOR_N_SENSORS];


  // Sync pulse timestamp estimation
  uint32_t lastSync;        // Last sync seen
  uint64_t currentSyncSum;  // Sum of the timestamps of all the close-together sync
  int nSyncPulses;          // Number of sync pulses accumulated

  // Sync pulse timestamps
  uint32_t currentSync;   // Sync currently used for sweep phase measurement
  uint32_t currentSync0;  // Sync0 of the current frame
  uint32_t currentSync0Width;  // Width of sync0 in the current frame
  uint32_t currentSync1Width;  // Width of sync1 in the current frame

  uint32_t currentSync0X;
  uint32_t currentSync0Y;
  uint32_t currentSync1X;
  uint32_t currentSync1Y;

  float frameWidth[2][2];

  // Base station and axis of the current frame
  int currentBaseStation;
  SweepId_t currentAxis;

  // Sweep timestamps
  struct {
    uint32_t timestamp;
    SweepStorageState_t state;
  } sweeps[PULSE_PROCESSOR_N_SENSORS];
  bool sweepDataStored;
} pulseProcessorV1_t;


/**
 * @brief Raw pulse data from the sensors. Data for pulses that are close in time and probably
 * comes from the same sweep. May contain pulse data from multiple base stations.
 *
 */
typedef struct {
    pulseProcessorFrame_t slots[PULSE_PROCESSOR_N_WORKSPACE];
    int slotsUsed;
    uint32_t latestTimestamp;
} pulseProcessorV2PulseWorkspace_t;

/**
 * @brief Holds derived data for one sweep through all sensors, derived from the pulses
 *
 */
typedef struct {
    uint32_t offset[PULSE_PROCESSOR_N_SENSORS];
    uint32_t timestamp0; // Timestamp of when the rotor has offset 0 (0 degrees)
    uint8_t channel;
} pulseProcessorV2SweepBlock_t;

/**
 * @brief Blocks used when decoding the pulse workspace
 *
 */
typedef struct {
    pulseProcessorV2SweepBlock_t blocks[PULSE_PROCRSSOR_N_CONCURRENT_BLOCKS];
} pulseProcessorV2BlockWorkspace_t;

/**
 * @brief Holds data for V2 base station decoding
 *
 */
typedef struct {
  pulseProcessorV2PulseWorkspace_t pulseWorkspace;
  pulseProcessorV2BlockWorkspace_t blockWorkspace;

  // Latest block from each base station. Used to pair both blocks (sweeps) from one rotaion of the rotor.
  pulseProcessorV2SweepBlock_t blocks[PULSE_PROCESSOR_N_BASE_STATIONS];

  // Timestamp of the rotor zero position for the latest processed slowbit
  uint32_t ootxTimestamps[PULSE_PROCESSOR_N_BASE_STATIONS];
} pulseProcessorV2_t;

typedef struct pulseProcessor_s {
  bool receivedBsSweep[PULSE_PROCESSOR_N_BASE_STATIONS];

  union {
    struct {
      pulseProcessorV1_t v1;
    };

    struct {
      pulseProcessorV2_t v2;
    };
  };

  ootxDecoderState_t ootxDecoder[PULSE_PROCESSOR_N_BASE_STATIONS];
  lighthouseCalibration_t bsCalibration[PULSE_PROCESSOR_N_BASE_STATIONS];
  baseStationGeometry_t bsGeometry[PULSE_PROCESSOR_N_BASE_STATIONS];
  baseStationGeometryCache_t bsGeoCache[PULSE_PROCESSOR_N_BASE_STATIONS];

  // Health check data
  uint32_t healthFirstSensorTs;
  uint8_t healthSensorBitField;
  bool healthDetermined;
} pulseProcessor_t;

typedef struct {
  float angles[PULSE_PROCESSOR_N_SWEEPS];
  float correctedAngles[PULSE_PROCESSOR_N_SWEEPS];
  int validCount;
} pulseProcessorBaseStationMeasuremnt_t;

typedef struct {
  pulseProcessorBaseStationMeasuremnt_t baseStatonMeasurements[PULSE_PROCESSOR_N_BASE_STATIONS];
} pulseProcessorSensorMeasurement_t;

typedef struct {
  pulseProcessorSensorMeasurement_t sensorMeasurementsLh1[PULSE_PROCESSOR_N_SENSORS];
  pulseProcessorSensorMeasurement_t sensorMeasurementsLh2[PULSE_PROCESSOR_N_SENSORS];
  lighthouseBaseStationType_t measurementType;
} pulseProcessorResult_t;

/**
 * @brief Interface for processing of pulse data from the lighthouse
 *
 * @param state
 * @param frameData
 * @param baseStation
 * @param axis
 * @return true, angle, base station and axis are written
 * @return false, no valid result
 */
typedef bool (*pulseProcessorProcessPulse_t)(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis);

/**
 * @brief Apply calibration correction to all angles of all sensors for a particular baseStation
 *
 * @param state
 * @param angles
 * @param baseStation
 */
void pulseProcessorApplyCalibration(pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation);

void pulseProcessorClearOutdated(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation);

/**
 * @brief Clear the result struct for one base station when the data is processed and converted to measurements
 *
 * @param angles The result struct to clear
 * @param baseStation The base station
 */
void pulseProcessorProcessed(pulseProcessorResult_t* angles, int baseStation);

/**
 * @brief Clear the result struct for one base station when the sensor data invalidated
 *
 * @param angles The result struct to clear
 * @param baseStation The base station
 */
void pulseProcessorClear(pulseProcessorResult_t* angles, int baseStation);

/**
 * @brief Clear result struct when the sensor data is invalidated
 *
 * @param angles
 */
void pulseProcessorAllClear(pulseProcessorResult_t* angles);

/**
 * Get quality of angles reception of the basestations.
 * 0 means no angles, 255 means reception of all angles of all axis of all basestations.
 */
uint8_t pulseProcessorAnglesQuality();
