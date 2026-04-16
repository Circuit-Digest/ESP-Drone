#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
  struct { float x,y,z; } velocity;  // used by autonav for shape motion
  bool velocity_body;
  bool velocity_valid;
  uint16_t thrust;                   // used for altitude hold / landing
} setpoint_t;
