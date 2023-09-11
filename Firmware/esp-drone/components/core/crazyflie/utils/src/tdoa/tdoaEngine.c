/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * tdoaEngine.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with tdoaEngine.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*
Implementation of LPS TDoA Tag functionality

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-20 anchors are visible
in every point. The tag is attached to a physical object and the expected
velocity is a few m/s, this means that anchors are within range for a time
period of seconds.

The implementation must handle
1. An infinite number of anchors, where around 20 are visible at one time
2. Any anchor ids
3. Dynamically changing visibility of anchors over time
4. Random TX times from anchors with possible packet collisions and packet loss

*/

#include <string.h>

#define DEBUG_MODULE "TDOA_ENGINE"
#include "debug_cf.h"

#include "tdoaEngine.h"
#include "tdoaStats.h"
#include "clockCorrectionEngine.h"
#include "physicalConstants.h"

#define MEASUREMENT_NOISE_STD 0.15f

void tdoaEngineInit(tdoaEngineState_t* engineState, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm) {
  tdoaStorageInitialize(engineState->anchorInfoArray);
  tdoaStatsInit(&engineState->stats, now_ms);
  engineState->sendTdoaToEstimator = sendTdoaToEstimator;
  engineState->locodeckTsFreq = locodeckTsFreq;
  engineState->matchingAlgorithm = matchingAlgorithm;

  engineState->matching.offset = 0;
}

#define TRUNCATE_TO_ANCHOR_TS_BITMAP 0x00FFFFFFFF
static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

static void enqueueTDOA(const tdoaAnchorContext_t* anchorACtx, const tdoaAnchorContext_t* anchorBCtx, double distanceDiff, tdoaEngineState_t* engineState) {
  tdoaStats_t* stats = &engineState->stats;

  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff
  };

  if (tdoaStorageGetAnchorPosition(anchorACtx, &tdoa.anchorPosition[0]) && tdoaStorageGetAnchorPosition(anchorBCtx, &tdoa.anchorPosition[1])) {
    STATS_CNT_RATE_EVENT(&stats->packetsToEstimator);

    uint8_t idA = tdoaStorageGetId(anchorACtx);
    uint8_t idB = tdoaStorageGetId(anchorBCtx);
    if (idA == stats->anchorId && idB == stats->remoteAnchorId) {
      stats->tdoa = distanceDiff;
    }
    if (idB == stats->anchorId && idA == stats->remoteAnchorId) {
      stats->tdoa = -distanceDiff;
    }

    engineState->sendTdoaToEstimator(&tdoa, idA, idB);
  }
}

static bool updateClockCorrection(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, tdoaStats_t* stats) {
  bool sampleIsReliable = false;

  const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);

  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    double clockCorrectionCandidate = clockCorrectionEngineCalculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TRUNCATE_TO_ANCHOR_TS_BITMAP);
    sampleIsReliable = clockCorrectionEngineUpdate(tdoaStorageGetClockCorrectionStorage(anchorCtx), clockCorrectionCandidate);

    if (sampleIsReliable){
      if (tdoaStorageGetId(anchorCtx) == stats->anchorId) {
        stats->clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        STATS_CNT_RATE_EVENT(&stats->clockCorrectionCount);
      }
    }
  }

  return sampleIsReliable;
}

static int64_t calcTDoA(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx);

  const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

  const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx);

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_T =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T;
}

static double calcDistanceDiff(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq) {
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  return SPEED_OF_LIGHT * tdoa / locodeckTsFreq;
}

static bool matchRandomAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx) {
  engineState->matching.offset++;
  int remoteCount = 0;
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

  uint32_t now_ms = anchorCtx->currentTime_ms;

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
  for (int i = engineState->matching.offset; i < (remoteCount + engineState->matching.offset); i++) {
    uint8_t index = i % remoteCount;
    const uint8_t candidateAnchorId = engineState->matching.id[index];
    if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
      if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx) && tdoaStorageGetTimeOfFlight(anchorCtx, candidateAnchorId)) {
        return true;
      }
    }
  }

  otherAnchorCtx->anchorInfo = 0;
  return false;
}

static bool matchYoungestAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx) {
    int remoteCount = 0;
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

    uint32_t now_ms = anchorCtx->currentTime_ms;
    uint32_t youmgestUpdateTime = 0;
    int bestId = -1;

    for (int index = 0; index < remoteCount; index++) {
      const uint8_t candidateAnchorId = engineState->matching.id[index];
      if (tdoaStorageGetTimeOfFlight(anchorCtx, candidateAnchorId)) {
        if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
          uint32_t updateTime = otherAnchorCtx->anchorInfo->lastUpdateTime;
          if (updateTime > youmgestUpdateTime) {
            if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx)) {
              youmgestUpdateTime = updateTime;
              bestId = candidateAnchorId;
            }
          }
        }
      }
    }

    if (bestId >= 0) {
      tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, bestId, now_ms, otherAnchorCtx);
      return true;
    }

    otherAnchorCtx->anchorInfo = 0;
    return false;
}

static bool findSuitableAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx) {
  bool result = false;

  if (tdoaStorageGetClockCorrection(anchorCtx) > 0.0) {
    switch(engineState->matchingAlgorithm) {
      case TdoaEngineMatchingAlgorithmRandom:
        result = matchRandomAnchor(engineState, otherAnchorCtx, anchorCtx);
        break;

      case TdoaEngineMatchingAlgorithmYoungest:
        result = matchYoungestAnchor(engineState, otherAnchorCtx, anchorCtx);
        break;

      default:
        // Do nothing
        break;
    }
  }

  return result;
}

void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx)) {
    STATS_CNT_RATE_EVENT(&engineState->stats.contextHitCount);
  } else {
    STATS_CNT_RATE_EVENT(&engineState->stats.contextMissCount);
  }
}

void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
  if (timeIsGood) {
    STATS_CNT_RATE_EVENT(&engineState->stats.timeIsGood);

    tdoaAnchorContext_t otherAnchorCtx;
    if (findSuitableAnchor(engineState, &otherAnchorCtx, anchorCtx)) {
      STATS_CNT_RATE_EVENT(&engineState->stats.suitableDataFound);
      double tdoaDistDiff = calcDistanceDiff(&otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->locodeckTsFreq);
      enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, engineState);
    }
  }
}
