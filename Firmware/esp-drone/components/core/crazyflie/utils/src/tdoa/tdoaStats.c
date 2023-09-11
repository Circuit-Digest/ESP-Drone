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
 * lpsTdoaTagStats.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTagStats.c. If not, see <http://www.gnu.org/licenses/>.
 */

/*
Log statistics for the tdoa engine
*/

#include <string.h>

#include "tdoaStats.h"

#define STATS_INTERVAL 500


void tdoaStatsInit(tdoaStats_t* tdoaStats, uint32_t now_ms) {
  memset(tdoaStats, 0, sizeof(tdoaStats_t));
  tdoaStats->remoteAnchorId = tdoaStats->newRemoteAnchorId = 1;

  tdoaStats->nextStatisticsTime = now_ms + STATS_INTERVAL;
  tdoaStats->previousStatisticsTime = 0;

  STATS_CNT_RATE_INIT(&tdoaStats->packetsReceived, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->packetsToEstimator, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->clockCorrectionCount, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->contextHitCount, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->contextMissCount, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->timeIsGood, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->suitableDataFound, STATS_INTERVAL);
}

void tdoaStatsUpdate(tdoaStats_t* tdoaStats, uint32_t now_ms) {
  if (now_ms > tdoaStats->nextStatisticsTime) {
    if (tdoaStats->anchorId != tdoaStats->newAnchorId) {
      tdoaStats->anchorId = tdoaStats->newAnchorId;

      // Reset anchor stats
      tdoaStats->clockCorrection = 0.0;
      tdoaStats->tof = 0;
      tdoaStats->tdoa = 0;
    }

    if (tdoaStats->remoteAnchorId != tdoaStats->newRemoteAnchorId) {
      tdoaStats->remoteAnchorId = tdoaStats->newRemoteAnchorId;

      // Reset remote anchor stats
      tdoaStats->tof = 0;
      tdoaStats->tdoa = 0;
    }

    tdoaStats->previousStatisticsTime = now_ms;
    tdoaStats->nextStatisticsTime = now_ms + STATS_INTERVAL;
  }
}
