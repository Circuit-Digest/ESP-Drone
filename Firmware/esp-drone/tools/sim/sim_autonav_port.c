#include "sim_sensors.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "esp_timer.h"

/* monotonic microsecond timer */
uint64_t esp_timer_get_time(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

/* ---- sensor hooks expected by autonav.c ---- */
bool sensorsGetDownTofMm(uint16_t* out_mm) {
  static uint64_t t0 = 0; if (!t0) t0 = esp_timer_get_time();
  double t = (esp_timer_get_time() - t0)/1e6; // s
  double target = 1200.0;                     // mm
  double alt = target + 5.0 * sin(t * 3.0);   // tiny noise
  if (out_mm) *out_mm = (uint16_t)(alt < 0 ? 0 : alt);
  return true;
}
bool sensorsGetFrontTofMm(uint16_t* out_mm) {
  static uint64_t t0 = 0; if (!t0) t0 = esp_timer_get_time();
  double t = (esp_timer_get_time() - t0)/1e6; // s
  uint16_t d = 2000;                // clear
  if (t > 8.0 && t < 20.0) d = 600; // obstacle in front for 12s
  if (out_mm) *out_mm = d;
  return true;
}
