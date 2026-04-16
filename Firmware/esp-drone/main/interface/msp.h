/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 *
 * msp.h - Definitions for the MultiWii Serial Protocol
 *
 * The MultiWii Serial Protocol is a commonly used protocol for OSDs,
 * data logging, and communication with a ground telemetry station.
 * For more details, see the official MSP documentation and support thread:
 *  - http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
 *  - http://www.multiwii.com/forum/viewtopic.php?f=8&t=1516
 */

#ifndef MSP_H_
#define MSP_H_
#include <stdint.h>
#include <stdbool.h>

/**
 * Function signature for a response callback to be provided by a client
 */
typedef void (*MspResponseCallback)(uint8_t* pBuffer, uint32_t bufferLen);

/**
 * Structure describing the MSP packet header
 * This header is followed by a number of data bytes
 * and a CRC. The CRC is the XOR of size, command, and all
 * data bytes into a zeroed sum.
 */
typedef struct _MspHeader
{
  uint8_t preamble[2];    // Should always be '$M'
  uint8_t direction;
  uint8_t size;
  uint8_t command;
}__attribute__((packed)) MspHeader;

/**
 * Structure representing an instance of the MSP
 * library. State is contained in this object, so
 * there can be more than one MSP client in the system
 */
typedef struct
{
  // For storing MSP request data/state
  MspHeader requestHeader;
  uint8_t requestCrc;
  uint8_t requestState;

  // Response context
  uint8_t mspResponse[256];
  // Size of the complete response message contained
  // in the mspResponse buffer
  uint16_t mspResponseSize;

  // The client callback to be invoked
  // when a response message is ready
  MspResponseCallback responseCallback;

} MspObject;

/**
 * Initializes an instance of the MSP library
 * @param[in]   pMspObject      Pointer to the MSP object to initialize
 * @param[in]   callback        Function pointer to a callback
 */
void mspInit(MspObject* pMspObject, const MspResponseCallback callback);

/**
 * Processes the next byte received by the client
 * @param[in]   pMspObject      Pointer to the MSB object
 * @param[in]   data            The data to process
 */
void mspProcessByte(MspObject* pMspObject, const uint8_t data);

#endif /* MSP_H_ */
