/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 * clockCorrectionEngine.h - utitlity for keeping track of clock drift
 * in UWB positioning system.
 */

#ifndef clockCorrectionEngine_h
#define clockCorrectionEngine_h

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  double clockCorrection;
  unsigned int clockCorrectionBucket;
} clockCorrectionStorage_t;

double clockCorrectionEngineGet(const clockCorrectionStorage_t* storage);
double clockCorrectionEngineCalculate(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask);
bool clockCorrectionEngineUpdate(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate);

#endif /* clockCorrectionEngine_h */
