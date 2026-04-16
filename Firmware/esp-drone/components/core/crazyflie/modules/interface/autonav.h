#pragma once

#include <stdbool.h>
#include <stdint.h>

// --- States ---
typedef enum {
  AUTONAV_IDLE = 0,
  AUTONAV_RUNNING,
  AUTONAV_HOLD_OBSTACLE,
  AUTONAV_LANDING,
  AUTONAV_LANDED,
  AUTONAV_OVERRIDE
} autonav_state_t;

// --- Shape IDs (for UI/commands) ---
typedef enum {
  AUTONAV_SHAPE_STOP = 0,
  AUTONAV_SHAPE_SQUARE = 1,
  AUTONAV_SHAPE_RECTANGLE = 2,
  AUTONAV_SHAPE_OVAL = 3,
  AUTONAV_SHAPE_TRIANGLE = 4,
  AUTONAV_SHAPE_PENTAGON = 5
} autonav_shape_t;

// API
void autonavInit(void);
void autonavUpdate(uint32_t tickMs);

void autonavStartShape(uint8_t shapeId);   // 0 = stop, others = shapes
void autonavKickSafety(void);
void autonavSetObstacle(bool detected);

autonav_state_t autonavGetState(void);

// Manual override API
void autonavEnableManualOverride(bool enable);
void autonavEnterOverride(void);
bool autonavIsOverride(void);
