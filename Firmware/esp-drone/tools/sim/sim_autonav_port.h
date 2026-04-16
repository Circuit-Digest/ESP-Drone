#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ---- minimal stand-ins for firmware types ---- */
typedef struct {
  /* match what autonav.c uses: thrust and (optionally) velocity */
  uint16_t thrust;
  struct { float x, y, z; } velocity; /* safe to exist even if unused */
} setpoint_t;

void commanderSetSetpoint(setpoint_t* sp, uint32_t tickMs);

/* time in microseconds */
uint64_t sim_now_us(void);

/* sensor hooks that your autonav.c expects */
bool sensorsGetDownTofMm(uint16_t* out_mm);
bool sensorsGetFrontTofMm(uint16_t* out_mm);

/* logging helpers */
void sim_log(const char* tag, const char* fmt, ...);

/* expose AUTONAV state enum for printing (declare same as your autonav.h) */
typedef enum {
  AUTONAV_IDLE = 0,
  AUTONAV_RUNNING,
  AUTONAV_HOLD_OBSTACLE,
  AUTONAV_LANDING,
  AUTONAV_LANDED,
} autonav_state_t;

/* autonav API (from your autonav.h) */
void autonavInit(void);
void autonavUpdate(uint32_t tickMs);
void autonavStartShape(uint8_t shapeId);
void autonavKickSafety(void);
autonav_state_t autonavGetState(void);
void autonavSetTargetAltMm(uint16_t mm);
