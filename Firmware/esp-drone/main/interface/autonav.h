#pragma once

#include <stdbool.h>
#include <stdint.h>

// ---- Autonav States ----
typedef enum {
    AUTONAV_IDLE = 0,
    AUTONAV_RUNNING,
    AUTONAV_HOLD_OBSTACLE,
    AUTONAV_LANDING,
    AUTONAV_LANDED
} autonav_state_t;

// ---- Public API ----

// Initialize the autonomous navigation module
void autonavInit(void);

// Called periodically from the stabilizer loop
void autonavUpdate(uint32_t tickMs);

// Start a shape flight (0 = stop, 1 = square, 2 = circle, etc.)
void autonavStartShape(uint8_t shapeId);

// Stop navigation and reset state
void autonavStop(void);

// Reset the safety timer (called when any command is received from app/RC)
void autonavKickSafety(void);

// Set or query target altitude
void autonavSetTargetAltMm(uint16_t mm);
autonav_state_t autonavGetState(void);

// Obstacle sensor hook (optional: for unit test / external injection)
void autonavSetObstacle(bool detected);
