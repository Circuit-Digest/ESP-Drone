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
 * pulse_processor_v2.h - pulse decoding for lighthouse V2 base stations
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "pulse_processor.h"


/**
 * @brief Process pulse data from a lighthouse V2 system
 *
 * @param state
 * @param frameData
 * @param baseStation
 * @param axis
 * @return true, angle, base station and axis are written
 * @return false, no valid result
 */
bool pulseProcessorV2ProcessPulse(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis);

/**
 * @brief Convert Lighthouse v2 angles to Lighthouse V1 angles
 *
 * @param v2Angle1 First LH V2 angle
 * @param v2Angle2 Second LH V2 angle
 * @param v1Angles The resulting V1 angles
 */
void pulseProcessorV2ConvertToV1Angles(const float v2Angle1, const float v2Angle2, float* v1Angles);

uint8_t pulseProcessorV2AnglesQuality();
