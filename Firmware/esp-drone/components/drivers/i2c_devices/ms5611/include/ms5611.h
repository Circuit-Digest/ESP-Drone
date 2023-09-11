/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
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
 * @file ms5611.h
 * Driver for the ms5611 pressure sensor from measurement specialties.
 * Datasheet at http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */
#ifndef MS5611_H
#define MS5611_H

#include <stdbool.h>
#include "i2cdev.h"

// addresses of the device
#define MS5611_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS5611_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
#define MS5611_OSR_DEFAULT MS5611_OSR_4096

#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// Self test parameters. Only checks that values are sane
#define MS5611_ST_PRESS_MAX   (1100.0) //mbar
#define MS5611_ST_PRESS_MIN   (450.0)  //mbar
#define MS5611_ST_TEMP_MAX    (60.0)   //degree celcius
#define MS5611_ST_TEMP_MIN    (-20.0)  //degree celcius

// Constants used to determine altitude from pressure
#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958 //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f


bool ms5611Init(I2C_Dev *i2cPort);
bool ms5611SelfTest(void);
bool ms5611EvaluateSelfTest(float min, float max, float value, char *string);
float ms5611GetPressure(uint8_t osr);
float ms5611CalcPressure(int32_t rawPress, int32_t dT);
float ms5611GetTemperature(uint8_t osr);
float ms5611CalcTemp(int32_t deltaT);
int32_t ms5611GetDeltaTemp(uint8_t osr);
int32_t ms5611CalcDeltaTemp(int32_t rawTemp);
int32_t ms5611RawPressure(uint8_t osr);
int32_t ms5611RawTemperature(uint8_t osr);
bool ms5611ReadPROM();
void ms5611Reset();
void ms5611StartConversion(uint8_t command);
int32_t ms5611GetConversion(uint8_t command);

void ms5611GetData(float *pressure, float *temperature, float *asl);
float ms5611PressureToAltitude(float *pressure);
#endif // MS5611_H
