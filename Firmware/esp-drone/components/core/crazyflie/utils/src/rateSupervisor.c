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
 * rateSupervisor.c - functionality to supervise the rate of modules
 */

#include "rateSupervisor.h"

void rateSupervisorInit(rateSupervisor_t* context, const uint32_t osTimeMs, const uint32_t evaluationIntervalMs, const uint32_t minCount, const uint32_t maxCount, const uint8_t skip) {
    context->count = 0;
    context->evaluationIntervalMs = evaluationIntervalMs;
    context->expectedMin = minCount;
    context->expectedMax = maxCount;
    context->nextEvaluationTimeMs = osTimeMs + evaluationIntervalMs;
    context->latestCount = 0;
    context->skip = skip;
}

bool rateSupervisorValidate(rateSupervisor_t* context, const uint32_t osTimeMs) {
    bool result = true;

    context->count += 1;
    if (osTimeMs > context->nextEvaluationTimeMs) {
        uint32_t actual = context->count;
        if (actual < context->expectedMin || actual > context->expectedMax) {
            result = false;
        }

        context->latestCount = context->count;
        context->count = 0;
        context->nextEvaluationTimeMs = osTimeMs + context->evaluationIntervalMs;

        if (context->skip > 0) {
            result = true;
            context->skip -= 1;
        }
    }

    return result;
}

uint32_t rateSupervisorLatestCount(rateSupervisor_t* context) {
    return context->latestCount;
}
