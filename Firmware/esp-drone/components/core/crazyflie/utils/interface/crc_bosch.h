/**
 * Copyright (C) 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 */

#ifndef _crc_h
#define _crc_h

#include <inttypes.h>

#define FALSE   0
#define TRUE    !FALSE

/* select the CRC standard from the list that follows */
#define CRC32

/* CRC standard list */
#if defined(CRC8)

typedef uint8_t crc;

#define CRC_NAME                "CRC-8"
#define POLYNOMIAL              0x07
#define CHECK_VALUE             0x04
#define INITIAL_REMAINDER       0x00
#define FINAL_XOR_VALUE         0x00
#define REFLECT                 0

#elif defined(CRC16C)

typedef unsigned short crc;

#define CRC_NAME                "CRC-16/CCITT-FALSE"
#define POLYNOMIAL              0x1021
#define CHECK_VALUE             0x29B1
#define INITIAL_REMAINDER       0xFFFF
#define FINAL_XOR_VALUE         0x0000
#define REFLECT                 0

#elif defined(CRC16A)

typedef unsigned short crc;

#define CRC_NAME                "CRC-16/ARC"
#define POLYNOMIAL              0xA001
#define CHECK_VALUE             0xBB3D
#define INITIAL_REMAINDER       0x0000
#define FINAL_XOR_VALUE         0x0000
#define REFLECT                 1

#elif defined(CRC16B)

typedef unsigned short crc;

#define CRC_NAME                "CRC-16/BUYPASS"
#define POLYNOMIAL              0x8005
#define CHECK_VALUE             0xFEE8
#define INITIAL_REMAINDER       0x0000
#define FINAL_XOR_VALUE         0x0000
#define REFLECT                 0

#elif defined(CRC32)

typedef unsigned long crc;

#define CRC_NAME                "CRC-32"
#define POLYNOMIAL              0xEDB88320
#define CHECK_VALUE             0xCBF43926
#define INITIAL_REMAINDER       0xFFFFFFFF
#define FINAL_XOR_VALUE         0xFFFFFFFF
#define RESIDUE                 0xDEBB20e3
#define REFLECT                 1

#elif defined(CRC32B)

typedef unsigned long crc;

#define CRC_NAME                "CRC-32/BZIP2"
#define POLYNOMIAL              0x04C11DB7
#define CHECK_VALUE             0xFC891918
#define INITIAL_REMAINDER       0xFFFFFFFF
#define FINAL_XOR_VALUE         0xFFFFFFFF
#define RESIDUE                 0xC704DD7B
#define REFLECT                 0

#else

#error "one of CRC standard has to be #define'd."

#endif

#define WIDTH           (8 * sizeof(crc))
#define TOPBIT         (1 << (WIDTH - 1))

void crcTableInit(crc* crcTable);
crc crcByBit(const uint8_t* message, uint32_t bytesToProcess,
             crc remainder, crc finalxor);
crc crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              crc remainder, crc finalxor, crc* crcTable);

#endif /* _crc_h */
