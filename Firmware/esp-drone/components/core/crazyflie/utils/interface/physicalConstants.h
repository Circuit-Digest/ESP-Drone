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
 */

#pragma once

#define SPEED_OF_LIGHT (299792458.0)
#define GRAVITY_MAGNITUDE (9.81f)

#ifndef M_PI
  #define M_PI   3.14159265358979323846
#endif

#ifndef M_PI_F
  #define M_PI_F   (3.14159265358979323846f)
#endif

#ifndef M_1_PI_F
  #define M_1_PI_F (0.31830988618379067154f)
#endif

#ifndef M_PI_2_F
  #define M_PI_2_F (1.57079632679f)
#endif

#ifndef CF_MASS
  #define CF_MASS (0.027f) // in kg
#endif
