/**
 *
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * Version.h - Firmware version
 */
#ifndef __VERSION_H__
#define __VERSION_H__

#include <stdbool.h>

extern const char * V_SLOCAL_REVISION;
extern const char * V_SREVISION;
extern const char * V_STAG;
extern const bool V_MODIFIED;
extern const bool V_PRODUCTION_RELEASE;

#endif /* __VERSION_H__ */
