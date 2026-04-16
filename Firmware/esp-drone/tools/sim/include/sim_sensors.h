#pragma once
#include <stdint.h>
#include <stdbool.h>

// Simulated sensor APIs for autonav
bool sensorsGetDownTofMm(uint16_t* out_mm);
bool sensorsGetFrontTofMm(uint16_t* out_mm);
