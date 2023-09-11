#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "peer_localization.h"


void peerLocalizationInit()
{
  // All other_positions[in].id will be set to zero due to static initialization.
  // If we ever switch to dynamic allocation, we need to set them to zero explicitly.
}

bool peerLocalizationTest()
{
  return true;
}

// array of other's position
static peerLocalizationOtherPosition_t other_positions[PEER_LOCALIZATION_MAX_NEIGHBORS];

bool peerLocalizationTellPosition(int cfid, positionMeasurement_t const *pos)
{
  for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    if (other_positions[i].id == 0 || other_positions[i].id == cfid) {
      other_positions[i].id = cfid;
      other_positions[i].pos.x = pos->x;
      other_positions[i].pos.y = pos->y;
      other_positions[i].pos.z = pos->z;
      other_positions[i].pos.timestamp = xTaskGetTickCount();
      return true;
    }
  }
  return false;
}

bool peerLocalizationIsIDActive(uint8_t cfid)
{
  for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    if (other_positions[i].id == cfid) {
      return true;
    }
  }
  return false;
}

peerLocalizationOtherPosition_t *peerLocalizationGetPositionByID(uint8_t cfid)
{
  for (uint8_t i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    if (other_positions[i].id == cfid) {
      return &other_positions[i];
    }
  }
  return NULL;
}

peerLocalizationOtherPosition_t *peerLocalizationGetPositionByIdx(uint8_t idx)
{
  // TODO: should we return NULL if the id == 0?
  if (idx < PEER_LOCALIZATION_MAX_NEIGHBORS) {
    return &other_positions[idx];
  }
  return NULL;
}
