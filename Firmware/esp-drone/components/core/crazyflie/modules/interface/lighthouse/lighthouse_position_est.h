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
 * lighthouse_position_est.h - position estimaton for the lighthouse system
 */

#pragma once

#include "pulse_processor.h"
#include "lighthouse_geometry.h"


void lighthousePositionEstInit();

/**
 * @brief Set new geometry data for one base station
 *
 * @param baseStation   The id of the base station
 * @param geometry      The geometry data
 */
void lighthousePositionSetGeometryData(const uint8_t baseStation, const baseStationGeometry_t* geometry);

void lighthousePositionEstimatePoseCrossingBeams(const pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation);
void lighthousePositionEstimatePoseSweeps(const pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation);
