/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * statsCnt.c - utitlity for logging rates
 */

#include "statsCnt.h"
#include "debug_cf.h"


void statsCntRateCounterInit(statsCntRateCounter_t* counter, uint32_t averagingIntervalMs) {
    counter->intervalMs = averagingIntervalMs;
    counter->count = 0;
    counter->latestCount = 0;
    counter->latestAveragingMs = 0;
    counter->latestRate = 0.0f;
}

float statsCntRateCounterUpdate(statsCntRateCounter_t* counter, uint32_t now_ms) {
    uint32_t dt_ms = now_ms - counter->latestAveragingMs;
    if (dt_ms > counter->intervalMs) {
        float dt_s = dt_ms / 1000.0f;
        float dv = counter->count - counter->latestCount;

        counter->latestRate = dv / dt_s;

        counter->latestCount = counter->count;
        counter->latestAveragingMs = now_ms;
    }

    return counter->latestRate;
}

void statsCntRateLoggerInit(statsCntRateLogger_t* logger, uint32_t averagingIntervalMs) {
    statsCntRateCounterInit(&logger->rateCounter, averagingIntervalMs);

    logger->logByFunction.data = (void*)logger;
    logger->logByFunction.aquireFloat = statsCntRateLogHandler;
}

float statsCntRateLogHandler(uint32_t timestamp, void* data) {
    statsCntRateLogger_t* logger = (statsCntRateLogger_t*)data;
    return statsCntRateCounterUpdate(&logger->rateCounter, timestamp);
}
