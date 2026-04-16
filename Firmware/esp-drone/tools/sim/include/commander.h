#pragma once
#include <stdint.h>
#include <stdio.h>
#include "stabilizer.h"

static inline void commanderSetSetpoint(const setpoint_t* sp, uint32_t tickMs) {
  static uint32_t last = 0;
  if ((tickMs/1000) != (last/1000)) {
    printf("tick=%5u s | v=(%.2f, %.2f, %.2f) thrust=%u\n",
           tickMs/1000, sp->velocity.x, sp->velocity.y, sp->velocity.z, sp->thrust);
  }
  last = tickMs;
}
