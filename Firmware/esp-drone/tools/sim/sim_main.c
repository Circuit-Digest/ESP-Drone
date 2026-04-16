#include "sim_sensors.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "autonav.h"
#define SAFETY_TIMEOUT_MS 10000  // 10s for sim
int main(void) {
  autonavInit();
  autonavSetTargetAltMm(1200);   // 1.2 m
  autonavStartShape(1);          // e.g., square
  autonavKickSafety();

  const int dt_ms = 10; // 100 Hz
  for (int t=0; t<30000; t+=dt_ms) {
    autonavUpdate((uint32_t)t);
    // if ((t % 2000) == 0) autonavKickSafety(); // keep safety alive
    if ((t % 1000) == 0) {
  uint16_t down=0, front=0;
  sensorsGetDownTofMm(&down);
  sensorsGetFrontTofMm(&front);
  printf("[t=%5d ms] state=%d down=%umm front=%umm\n",
         t, (int)autonavGetState(), down, front);
}
    usleep(dt_ms * 1000);
  }
  puts("=== sim done ===");
  return 0;
}
