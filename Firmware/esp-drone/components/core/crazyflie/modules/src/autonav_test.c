#include "autonav.h"
#include <stdio.h>
#include <unistd.h>   // sleep()

// Stubs for external functions
bool sensorsGetDownTofMm(uint16_t* out_mm) {
    *out_mm = 1200;   // Always report 1.2m
    return true;
}

bool sensorsGetFrontTofMm(uint16_t* out_mm) {
    static int counter = 0;
    counter++;
    if (counter < 50) {
        *out_mm = 2000;   // No obstacle initially
    } else {
        *out_mm = 500;    // After 50 cycles, simulate obstacle <0.8m
    }
    return true;
}

void commanderSetSetpoint(setpoint_t* sp, uint32_t tickMs) {
    printf("[%5d ms] Thrust=%u, State=%d\n", tickMs, sp->thrust, autonavGetState());
}

int main(void) {
    autonavInit();
    autonavStartShape(1);

    for (int t = 0; t < 70000; t += 100) {  // simulate 70s in 100ms steps
        autonavUpdate(t);

        // Kick safety until 20s, then stop → triggers 30s timeout
        if (t < 20000) {
            autonavKickSafety();
        }

        usleep(10000);  // slow down output
    }

    return 0;
}
