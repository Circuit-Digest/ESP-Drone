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
 * pulse_processor.c - pulse decoding for lighthouse V1 base stations
 *
 */

#include "pulse_processor.h"


void pulseProcessorApplyCalibration(pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation)
{
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[baseStation];
    lighthouseCalibrationApply(&state->bsCalibration[baseStation], bsMeasurement->angles, bsMeasurement->correctedAngles);
  }
}

/**
 * @brief Clear result struct
 *
 * @param angles
 */
void pulseProcessorClear(pulseProcessorResult_t* angles, int baseStation)
{
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    angles->sensorMeasurements[sensor].baseStatonMeasurements[baseStation].validCount = 0;
  }
}
