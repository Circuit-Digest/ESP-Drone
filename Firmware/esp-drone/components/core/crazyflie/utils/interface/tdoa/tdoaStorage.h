#ifndef __TDOA_STORAGE_H__
#define __TDOA_STORAGE_H__

#include "stabilizer_types.h"
#include "clockCorrectionEngine.h"

#define ANCHOR_STORAGE_COUNT 16
#define REMOTE_ANCHOR_DATA_COUNT 16
#define TOF_PER_ANCHOR_COUNT 16


typedef struct {
  uint8_t id; // Id of remote remote anchor
  uint8_t seqNr; // Sequence number of the packet received in the remote anchor (7 bits)
  int64_t rxTime; // Receive time of packet from anchor id in the remote anchor, in remote DWM clock
  uint32_t endOfLife;
} tdoaRemoteAnchorData_t;

typedef struct {
  uint8_t id;
  int64_t tof;
  uint32_t endOfLife; // Time stamp when the tof data is outdated, local system time in ms
} tdoaTimeOfFlight_t;

typedef struct {
  bool isInitialized;
  uint32_t lastUpdateTime; // The time when this anchor was updated the last time
  uint8_t id; // Anchor id

  int64_t txTime; // Transmit time of last packet, in remote DWM clock
  int64_t rxTime; // Receive time of last packet, in local DWM clock
  uint8_t seqNr; // Sequence nr of last packet (7 bits)

  clockCorrectionStorage_t clockCorrectionStorage;

  point_t position; // The coordinates of the anchor

  tdoaTimeOfFlight_t tof[TOF_PER_ANCHOR_COUNT];
  tdoaRemoteAnchorData_t remoteAnchorData[REMOTE_ANCHOR_DATA_COUNT];
} tdoaAnchorInfo_t;

typedef tdoaAnchorInfo_t tdaoAnchorInfoArray_t[ANCHOR_STORAGE_COUNT];


// The anchor context is used to pass information about an anchor as well as
// the current time to functions.
// The context should not be stored.
typedef struct {
  tdoaAnchorInfo_t* anchorInfo;
  uint32_t currentTime_ms;
} tdoaAnchorContext_t;


void tdoaStorageInitialize(tdoaAnchorInfo_t anchorStorage[]);

bool tdoaStorageGetCreateAnchorCtx(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
bool tdoaStorageGetAnchorCtx(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
uint8_t tdoaStorageGetListOfAnchorIds(tdoaAnchorInfo_t anchorStorage[], uint8_t unorderedAnchorList[], const int maxListSize);
uint8_t tdoaStorageGetListOfActiveAnchorIds(tdoaAnchorInfo_t anchorStorage[], uint8_t unorderedAnchorList[], const int maxListSize, const uint32_t currentTime_ms);

uint8_t tdoaStorageGetId(const tdoaAnchorContext_t* anchorCtx);
int64_t tdoaStorageGetRxTime(const tdoaAnchorContext_t* anchorCtx);
int64_t tdoaStorageGetTxTime(const tdoaAnchorContext_t* anchorCtx);
uint8_t tdoaStorageGetSeqNr(const tdoaAnchorContext_t* anchorCtx);
uint32_t tdoaStorageGetLastUpdateTime(const tdoaAnchorContext_t* anchorCtx);
clockCorrectionStorage_t* tdoaStorageGetClockCorrectionStorage(const tdoaAnchorContext_t* anchorCtx);
bool tdoaStorageGetAnchorPosition(const tdoaAnchorContext_t* anchorCtx, point_t* position);
void tdoaStorageSetAnchorPosition(tdoaAnchorContext_t* anchorCtx, const float x, const float y, const float z);
void tdoaStorageSetRxTxData(tdoaAnchorContext_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr);
double tdoaStorageGetClockCorrection(const tdoaAnchorContext_t* anchorCtx);
int64_t tdoaStorageGetRemoteRxTime(const tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor);
void tdoaStorageSetRemoteRxTime(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr);
void tdoaStorageGetRemoteSeqNrList(const tdoaAnchorContext_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]);
int64_t tdoaStorageGetTimeOfFlight(const tdoaAnchorContext_t* anchorCtx, const uint8_t otherAnchor);
void tdoaStorageSetTimeOfFlight(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof);

// Mainly for test
bool tdoaStorageIsAnchorInStorage(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor);

#endif // __TDOA_STORAGE_H__
