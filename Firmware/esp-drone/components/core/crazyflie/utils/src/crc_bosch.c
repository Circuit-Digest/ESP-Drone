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

#include "crc_bosch.h"

/* bit-wise crc calculation */
crc crcByBit(const uint8_t* message, uint32_t bytesToProcess,
             crc remainder, crc finalxor)
{
  for (unsigned int byte = 0; byte < bytesToProcess; ++byte)
    {
#if REFLECT
      remainder ^= *(message+byte);
#else
      remainder ^= ( *(message+byte) << (WIDTH - 8) );
#endif

      for(uint8_t bit = 8; bit > 0; --bit)
        {
#if REFLECT
          /* reflect is realized by mirroring algorithm
           * LSB is first to be processed */
          if (remainder & 1)
            remainder = (remainder >> 1) ^ POLYNOMIAL;
          else
            remainder = (remainder >> 1);
#else
          /* MSB is first to be processed */
          if (remainder & TOPBIT)
            remainder = (remainder << 1) ^ POLYNOMIAL;
          else
            remainder = (remainder << 1);
#endif
        }
    }
  return (remainder ^ finalxor);
}

/* byte-wise crc calculation, requires an initialized crcTable
 * this is factor 8 faster and should be used if multiple crcs
 * have to be calculated */
crc crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              crc remainder, crc finalxor, crc* crcTable)
{
  static uint8_t data;
  for (int byte = 0; byte < bytesToProcess; ++byte)
    {
#if REFLECT
      data = (*(message+byte) ^ remainder);
      remainder = *(crcTable+data) ^ (remainder >> 8);
#else
      data = ( *(message+byte) ^ (remainder >> (WIDTH - 8)) );
      remainder = *(crcTable+data) ^ (remainder << 8);
#endif
    }
  return (remainder ^ finalxor);
}

/* creates a lookup-table which is necessary for the crcByByte function */
void crcTableInit(crc* crcTable)
{
  uint8_t dividend = ~0;
  /* fill the table by bit-wise calculations of checksums
   * for each possible dividend */
  do {
      *(crcTable+dividend) = crcByBit(&dividend, 1, 0, 0);
  } while(dividend-- > 0);
}
