/**
 *
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2012-2016 Bitcraze AB
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
 * num.c - 16bit floating point handling functions
 */

#include <math.h>
#include <stdint.h>

#include "num.h"

/* Half precision floating point **********************************************
 *
 * To not use the GCC implementation, uint16_t is used to carry fp16 values
 *
 * FP16 or Half precision floating points is specified by IEEE 754 as binary 16.
 * (float is specified as binary 32). This implementation is NOT GUARANTEED to
 * be conform to the ieee 754 specification, it is 'just' good enough for the
 * Crazyflie usage. For more info about fp16 see
 * http://en.wikipedia.org/wiki/Half-precision_floating-point_format
 *
 * The current implementation has the following limitation:
 *  * No subnormalized number generation
 *  * Rounding seems to give at least 11 bits precision
 *  * Faster and smaller than the GCC implementation
 */

uint16_t single2half(float number)
{
    uint32_t num = *((uint32_t*)&number);
    uint32_t s = num>>31;
    uint32_t e = (num>>23)&0x0FF;

    if ((e==255) && (num&0x007fffff))
        return 0x7E00; // NaN
    if (e>(127+15))
        return s?0xFC00:0x7C00;  //+/- inf
    if (e<(127-15))
        return 0; //Do not handle generating subnormalised representation

    return (s<<15) | ((e-127+15)<<10) | (((num>>13)&0x3FF)+((num>>12)&0x01));
}

float half2single(uint16_t number)
{
    uint32_t fp32;
    uint32_t s = number>>15;
    uint32_t e = (number>>10)&0x01F;

    //All binary16 can be mapped in a binary32
    if(e==0)
        e=15-127;
    if (e==0x1F)
    {
        if (number&0x03FF)
            fp32 = 0x7FC00000; // NaN
        else
            fp32 = s?0xFF800000:0x7F800000;  //+/- inf
    }
    else
        fp32 = (s<<31) | ((e+127-15)<<23) | ((number&0x3ff)<<13);

    return *(float*)&fp32;
}


/*****************************************************************************/

uint16_t limitUint16(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal,value));
}

float deadband(float value, const float threshold)
{
  if (fabsf(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}
