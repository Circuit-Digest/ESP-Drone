/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 * rateSupervisor.h - functionality to supervise the rate of modules
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t count;
    uint32_t expectedMin;
    uint32_t expectedMax;
    uint32_t nextEvaluationTimeMs;
    uint32_t evaluationIntervalMs;
    uint32_t latestCount;
    uint8_t skip;
} rateSupervisor_t;

/**
 * @brief Initialize a rateSupervisor_t struct for rate measurements
 *
 * @param context The struct to initialize
 * @param osTimeMs The current os time in ms
 * @param evaluationIntervalMs How often to evaluate the rate, in ms
 * @param minCount The minimum number of validations we expect every evaluation interval
 * @param maxCount The maximum number of validations we expect every evaluation interval
 * @param skip The number of inital evaluations to ignore failures for. This is a convenient way to let the system warm up a bit before reporting problems.
 */
void rateSupervisorInit(rateSupervisor_t* context, const uint32_t osTimeMs, const uint32_t evaluationIntervalMs, const uint32_t minCount, const uint32_t maxCount, const uint8_t skip);

/**
 * @brief Validate the rate for a process. This function should be called from the process for which the rate
 * is to be supervised. When the function is called a counter is increased, and if the evaluation period
 * has passed, the rate is evaluated.
 *
 * @param context A rateSupervisor_t
 * @param osTimeMs The current os time in ms
 * @return true if the measured rate is within bounds, or we have not yet reached the next evaluation time
 * @return false if the measured rate is too low or high
 */
bool rateSupervisorValidate(rateSupervisor_t* context, const uint32_t osTimeMs);

/**
 * @brief Get the latest count. Useful to display the count after a failed validation.
 *
 * @param context A rateSupervisor_t
 * @return uint32_t The count at the latest evaluation time
 */
uint32_t rateSupervisorLatestCount(rateSupervisor_t* context);
