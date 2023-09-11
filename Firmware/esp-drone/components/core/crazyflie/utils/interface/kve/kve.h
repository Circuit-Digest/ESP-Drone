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
 * kveStorage.h - Low level storage functions
 *
 */

#pragma once

#include "kve/kve_common.h"

#include <stddef.h>
#include <stdbool.h>

void kveDefrag(kveMemory_t *kve);

bool kveStore(kveMemory_t *kve, char* key, const void* buffer, size_t length);

size_t kveFetch(kveMemory_t *kve, const char* key, void* buffer, size_t bufferLength);

bool kveDelete(kveMemory_t *kve, char* key);

void kveFormat(kveMemory_t *kve);

bool kveCheck(kveMemory_t *kve);
