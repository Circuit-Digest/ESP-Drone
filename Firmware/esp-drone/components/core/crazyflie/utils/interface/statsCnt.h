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
 * statsCnt.h - utitlity for logging rates
 */

#pragma once

#include <stdint.h>
#include "log.h"

/**
 * @brief A struct used to track event rates
 */
typedef struct {
    uint32_t count;
    uint32_t latestCount;
    uint32_t latestAveragingMs;
    float latestRate;
    uint32_t intervalMs;
} statsCntRateCounter_t;

/**
 * @brief Initialize a statsCntRateLogger_t struct.
 *
 * @param logger The rste counter to initialize
 * @param averagingIntervalMs The interval (in ms) between rate calculations
 */
void statsCntRateCounterInit(statsCntRateCounter_t* counter, uint32_t averagingIntervalMs);

/**
 * @brief A new rate is calculated if the time since the previous calculation is longer
 * than the configured interval time.
 *
 * @param counter The rate counter to update
 * @param now_ms Current system time in ms
 * @return float The latest calculated rate
 */
float statsCntRateCounterUpdate(statsCntRateCounter_t* counter, uint32_t now_ms);


// Log module integration -------------------------------------------------------

/**
 * @brief Struct to use a rate counter together with the log usb system.
 */
typedef struct {
    // logByFunction_t must be the first element in this struct since pointers
    // to this struct are cast to logByFunction_t* and used by the log module
    logByFunction_t logByFunction;
    statsCntRateCounter_t rateCounter;
} statsCntRateLogger_t;

/**
 * @brief Macro to initialize a statsCntRateLogger_t struct.
 *
 * @param LOGGER A pointer to a statsCntRateLogger_t
 * @param INTERVAL_MS The interval (in ms) between calculations of the rate
 */
#define STATS_CNT_RATE_INIT(LOGGER, INTERVAL_MS) statsCntRateLoggerInit(LOGGER, INTERVAL_MS)

#define STATS_CNT_RATE_DEFINE(NAME, INTERVAL_MS) statsCntRateLogger_t NAME = {.logByFunction = {.data = &NAME, .aquireFloat = statsCntRateLogHandler}, .rateCounter = {.intervalMs = (INTERVAL_MS), .count = 0, .latestCount = 0, .latestAveragingMs = 0, .latestRate = 0}}

/**
 * @brief Macro to add an event to a statsCntRateLogger_t, that is to increase the internal counter
 *
 * @param LOGGER A pointer to a statsCntRateLogger_t
 */
#define STATS_CNT_RATE_EVENT(LOGGER) ((LOGGER)->rateCounter.count++)

/**
 * @brief Macro to add CNT events to a statsCntRateLogger_t, that is to increase the internal counter with CNT
 *
 * @param LOGGER A pointer to a statsCntRateLogger_t
 * @param CNT    Number of counts to add
 */
#define STATS_CNT_RATE_MULTI_EVENT(LOGGER, CNT) ((LOGGER)->rateCounter.count += CNT)

/**
 * @brief Macro to add a statsCntRateLogger_t as a rate log. Used in a similar way as
 * LOG_ADD() in a LOG_GROUP_START() - LOG_GROUP_STOP() block
 *
 * @param LOGGER A pointer to a statsCntRateLogger_t
 */
#define STATS_CNT_RATE_LOG_ADD(NAME, LOGGER) LOG_ADD_BY_FUNCTION(LOG_FLOAT, NAME, LOGGER)

/**
 * @brief Initialize a statsCntRateLogger_t struct.
 *
 * @param logger The rste counter to initialize
 * @param averagingIntervalMs The interval (in ms) between rate calculations
 */
void statsCntRateLoggerInit(statsCntRateLogger_t* logger, uint32_t averagingIntervalMs);

/**
 * @brief Handler function used by the log module to acquire log values.
 * Calls statsCntRateCounterUpdate() to update the logger and acqure the rate.
 *
 * @param timestamp Current system time,in ms
 * @param data Pointer to a statsCntRateLogger_t
 * @return float The latest calculated rate
 */
float statsCntRateLogHandler(uint32_t timestamp, void* data);
