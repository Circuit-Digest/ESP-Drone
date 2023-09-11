/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * msp.c: Implementation of the MultiWii Serial Protocol
 *
 * The MultiWii Serial Protocol is a commonly used protocol for OSDs,
 * data logging, and communication with a ground telemetry station.
 * For more details, see the official MSP documentation and support thread:
 *  - http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
 *  - http://www.multiwii.com/forum/viewtopic.php?f=8&t=1516
 */

#include "msp.h"
#include "sensfusion6.h"
#include "commander.h"
#define DEBUG_MODULE "MSP"
#include "debug_cf.h"

// MSP command IDs
#define MSP_STATUS    101
#define MSP_RC        105
#define MSP_ATTITUDE  108
#define MSP_BOXIDS    119

// Misc MSP header defines
#define MSP_PREAMBLE_0    '$'
#define MSP_PREAMBLE_1    'M'
#define MSP_DIRECTION_IN  '<'
#define MSP_DIRECTION_OUT '>'

// Sensor flag defines
#define BARO_PRESENT  (0x01)
#define MAG_PRESENT   (0x02)
#define GPS_PRESENT   (0x04)
#define SONAR_PRESENT (0x08)

typedef enum
{
  MSP_REQUEST_STATE_WAIT_FOR_START,
  MSP_REQUEST_STATE_PREAMBLE,
  MSP_REQUEST_STATE_DIRECTION,
  MSP_REQUEST_STATE_SIZE,
  MSP_REQUEST_STATE_COMMAND,
  MSP_REQUEST_STATE_CRC
} MSP_REQUEST_STATE;

/**
 * Structures describing data payloads for MSP responses
 * All data must be serialized (LSB first) and the cflie
 * is using a little-endian processor, so no byte swapping
 * required
 */
typedef struct _MspStatus
{
  uint16_t cycleTime; // Rate of main stabilizer loop
  uint16_t i2cErrors; // Number of I2C bus errors encountered
  uint16_t sensors;   // Bitmask of supported sensors (see sensor flag bitmasks)
  uint32_t flags;     // Bitmask of flags/boxes
  uint8_t currentSet; // ID of current active settings/config
}__attribute__((packed)) MspStatus;

typedef struct _MspAttitude
{
  int16_t angX;       // Range [-1800,1800], Units: 1/10th degrees
  int16_t angY;       // Range [-1800,1800], Units: 1/10th degrees
  int16_t heading;    // Range [-180, 180], Units: degrees
}__attribute__((packed)) MspAttitude;

typedef struct _MspRc
{
  uint16_t roll;      // Range [1000,2000]
  uint16_t pitch;     // Range [1000,2000]
  uint16_t yaw;       // Range [1000,2000]
  uint16_t throttle;  // Range [1000,2000] 
}__attribute__((packed)) MspRc;

// Helpers
static uint8_t mspComputeCrc(uint8_t* pBuffer, uint32_t bufferLen);
static bool mspIsRequestValid(MspObject* pMspObject);
static void mspProcessRequest(MspObject* pMspObject);

// Request handlers
static void mspHandleRequestMspStatus(MspObject* pMspObject);
static void mspHandleRequestMspRc(MspObject* pMspObject);
static void mspHandleRequestMspAttitude(MspObject* pMspObject);
static void mspHandleRequestMspBoxIds(MspObject* pMspObject);

void mspInit(MspObject* pMspObject, const MspResponseCallback callback)
{
  pMspObject->requestState = MSP_REQUEST_STATE_WAIT_FOR_START;
  pMspObject->responseCallback = callback;
}

void mspProcessByte(MspObject* pMspObject, const uint8_t data)
{
  // If the start token is received at any time,
  // immediately transition back to the first state
  if(data == MSP_PREAMBLE_0)
  {
    pMspObject->requestState = MSP_REQUEST_STATE_PREAMBLE;
    pMspObject->requestHeader.preamble[0] = data;
    return;
  }

  switch(pMspObject->requestState)
  {
  case MSP_REQUEST_STATE_WAIT_FOR_START:
    // Do nothing until the start token comes
    break;

  case MSP_REQUEST_STATE_PREAMBLE:
    pMspObject->requestHeader.preamble[1] = data;
    pMspObject->requestState = MSP_REQUEST_STATE_DIRECTION;
    break;

  case MSP_REQUEST_STATE_DIRECTION:
    pMspObject->requestHeader.direction = data;
    pMspObject->requestState = MSP_REQUEST_STATE_SIZE;
    break;

  case MSP_REQUEST_STATE_SIZE:
    pMspObject->requestHeader.size = data;
    pMspObject->requestState = MSP_REQUEST_STATE_COMMAND;
    break;

  case MSP_REQUEST_STATE_COMMAND:
    pMspObject->requestHeader.command = data;
    pMspObject->requestState = MSP_REQUEST_STATE_CRC;
    break;

  case MSP_REQUEST_STATE_CRC:
    pMspObject->requestCrc = data;

    // Have a completed request
    mspProcessRequest(pMspObject);

    pMspObject->requestState = MSP_REQUEST_STATE_WAIT_FOR_START;
    break;
  }
}

uint8_t mspComputeCrc(uint8_t* pBuffer, uint32_t bufferLen)
{
  uint8_t crc = 0;

  // Make sure the buffer is at least the size of a header
  if(bufferLen >= sizeof(MspHeader))
  {
    MspHeader* pHeader = (MspHeader*)pBuffer;

    // Make sure the buffer is at least the size of the buffer and data
    if(bufferLen >= sizeof(MspHeader) + pHeader->size)
    {
      // The MSP CRC is defined as the XOR of size, command, and all
      // data bytes into a zeroed sum.

      crc ^= pHeader->size;
      crc ^= pHeader->command;

      for(uint16_t i = 0; i < pHeader->size; i++)
      {
        crc ^= pBuffer[sizeof(MspHeader) + i];
      }
    }
  }

  return crc;
}

bool mspIsRequestValid(MspObject* pMspObject)
{
  if(pMspObject->requestHeader.preamble[0] != MSP_PREAMBLE_0 ||
      pMspObject->requestHeader.preamble[1] != MSP_PREAMBLE_1)
  {
    // Invalid preamble
    DEBUG_PRINTD("MSP Request has invalid preamble %d %d\n", pMspObject->requestHeader.preamble[0], pMspObject->requestHeader.preamble[1]);
    return false;
  }

  if(pMspObject->requestHeader.direction != MSP_DIRECTION_IN)
  {
    // Requests should be inputs
    DEBUG_PRINTD("MSP Request has invalid direction %d\n", pMspObject->requestHeader.direction);
    return false;
  }

  if(pMspObject->requestHeader.size != 0)
  {
    // Requests should not have a payload
    DEBUG_PRINTD("MSP Request has invalid size %d\n", pMspObject->requestHeader.size);
    return false;
  }

  if(pMspObject->requestCrc !=
      mspComputeCrc((uint8_t*)&pMspObject->requestHeader, sizeof(pMspObject->requestHeader)))
  {
    // CRC does not match
    DEBUG_PRINTD("MSP Request has invalid crc (%d != %d)\n", pMspObject->requestCrc, mspComputeCrc((uint8_t*)&pMspObject->requestHeader, sizeof(pMspObject->requestHeader)));
    return false;
  }

  return true;
}

void mspProcessRequest(MspObject* pMspObject)
{
  if(!mspIsRequestValid(pMspObject))
  {
    DEBUG_PRINT("Received invalid or improperly formatted MSP request\n");
    return;
  }

  switch(pMspObject->requestHeader.command)
  {
  case MSP_STATUS:
    mspHandleRequestMspStatus(pMspObject);

    if(pMspObject->responseCallback)
    {
      pMspObject->responseCallback(pMspObject->mspResponse, pMspObject->mspResponseSize);
    }
    break;

  case MSP_RC:
    mspHandleRequestMspRc(pMspObject);

    if(pMspObject->responseCallback)
    {
      pMspObject->responseCallback(pMspObject->mspResponse, pMspObject->mspResponseSize);
    }
    break;

  case MSP_ATTITUDE:
    mspHandleRequestMspAttitude(pMspObject);

    if(pMspObject->responseCallback)
    {
      pMspObject->responseCallback(pMspObject->mspResponse, pMspObject->mspResponseSize);
    }
    break;

  case MSP_BOXIDS:
    mspHandleRequestMspBoxIds(pMspObject);
    
    if(pMspObject->responseCallback)
    {
      pMspObject->responseCallback(pMspObject->mspResponse, pMspObject->mspResponseSize);
    }
    break;

  default:
    DEBUG_PRINT("Received unsupported MSP request: %d\n", pMspObject->requestHeader.command);
    break;
  }
}

void mspHandleRequestMspStatus(MspObject* pMspObject)
{
  MspHeader* pHeader = (MspHeader*)pMspObject->mspResponse;
  MspStatus* pData = (MspStatus*)(pMspObject->mspResponse + sizeof(MspHeader));
  uint8_t* pCrc = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader) + sizeof(*pData));

  // Header
  pHeader->preamble[0] = MSP_PREAMBLE_0;
  pHeader->preamble[1] = MSP_PREAMBLE_1;
  pHeader->direction = MSP_DIRECTION_OUT;
  pHeader->size = sizeof(*pData);
  pHeader->command = MSP_STATUS;

  // Data
  pData->cycleTime = 1000; // TODO: API to query this?
  pData->i2cErrors = 0; // unused
  pData->sensors = 0x0001; // no sensors supported yet, but need to report at least one to get the level bars to show
  pData->flags = 0x00000001; // always report armed (bit zero)
  pData->currentSet = 0x00;

  // CRC
  *pCrc = mspComputeCrc(pMspObject->mspResponse, sizeof(pMspObject->mspResponse));

  // Update total response size
  pMspObject->mspResponseSize = sizeof(MspHeader) + sizeof(*pData) + 1;
}

void mspHandleRequestMspRc(MspObject* pMspObject)
{
  MspHeader* pHeader = (MspHeader*)pMspObject->mspResponse;
  MspRc* pData = (MspRc*)(pMspObject->mspResponse + sizeof(MspHeader));
  uint8_t* pCrc = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader) + sizeof(*pData));

  // Header
  pHeader->preamble[0] = MSP_PREAMBLE_0;
  pHeader->preamble[1] = MSP_PREAMBLE_1;
  pHeader->direction = MSP_DIRECTION_OUT;
  pHeader->size = sizeof(*pData);
  pHeader->command = MSP_RC;

  // TODO: get actual data - for now hardcode the midpoint
  pData->roll = 1500;
  pData->pitch = 1500;
  pData->yaw = 1500;
  pData->throttle = 1500;

  // CRC
  *pCrc = mspComputeCrc(pMspObject->mspResponse, sizeof(pMspObject->mspResponse));

  // Update total response size
  pMspObject->mspResponseSize = sizeof(MspHeader) + sizeof(*pData) + 1;
}

static void mspHandleRequestMspAttitude(MspObject* pMspObject)
{
  MspHeader* pHeader = (MspHeader*)pMspObject->mspResponse;
  MspAttitude* pData = (MspAttitude*)(pMspObject->mspResponse + sizeof(MspHeader));
  uint8_t* pCrc = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader) + sizeof(*pData));

  // Header
  pHeader->preamble[0] = MSP_PREAMBLE_0;
  pHeader->preamble[1] = MSP_PREAMBLE_1;
  pHeader->direction = MSP_DIRECTION_OUT;
  pHeader->size = sizeof(*pData);
  pHeader->command = MSP_ATTITUDE;

  // Data
  float roll;
  float pitch;
  float yaw;
  sensfusion6GetEulerRPY(&roll, &pitch, &yaw); // TODO: handle kalman estimator

  pData->angX = (int16_t)(roll * 10);
  pData->angY = (int16_t)(pitch * 10);
  pData->heading = 0; // TODO: mag support

  // CRC
  *pCrc = mspComputeCrc(pMspObject->mspResponse, sizeof(pMspObject->mspResponse));

  // Update total response size
  pMspObject->mspResponseSize = sizeof(MspHeader) + sizeof(*pData) + 1;
}

static void mspHandleRequestMspBoxIds(MspObject* pMspObject)
{
  MspHeader* pHeader = (MspHeader*)pMspObject->mspResponse;
  uint8_t* pData = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader));
  uint8_t* pCrc = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader) + sizeof(*pData));

  // Header
  pHeader->preamble[0] = MSP_PREAMBLE_0;
  pHeader->preamble[1] = MSP_PREAMBLE_1;
  pHeader->direction = MSP_DIRECTION_OUT;
  pHeader->size = sizeof(*pData);
  pHeader->command = MSP_BOXIDS;
  
  // TODO: Data - this needs to be properly implemented
  // For now, we just return byte 0 = 0 which tells 
  // the client to use box ID 0 for the ARMED box
  pData[0] = 0x00;

  // CRC
  *pCrc = mspComputeCrc(pMspObject->mspResponse, sizeof(pMspObject->mspResponse));

  // Update total response size
  pMspObject->mspResponseSize = sizeof(MspHeader) + sizeof(*pData) + 1;
}
