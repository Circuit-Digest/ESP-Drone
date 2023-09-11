/*
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2015 Bitcraze AB
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
 * deck_digital.h - Arduino-compatible digital I/O API
 */

#ifndef __DECK_DIGITAL_H__
#define __DECK_DIGITAL_H__

#include <stdint.h>

#define LOW 0x0
#define HIGH 0x1

#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3

void pinMode(uint32_t pin, uint32_t mode);

void digitalWrite(uint32_t pin, uint32_t val);

int digitalRead(uint32_t pin);

#endif
