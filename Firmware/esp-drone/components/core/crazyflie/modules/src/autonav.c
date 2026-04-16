#include "autonav.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "commander.h"          // commanderSetSetpoint(...)
#include "stabilizer.h"         // setpoint_t
#include "esp_timer.h"          // esp_timer_get_time()
#include <math.h>

// ---- CONFIG ----
#define AUTONAV_DEFAULT_ALT_MM        1200
#define SAFETY_TIMEOUT_MS             30000   // 30s no heartbeat -> land
#define OBSTACLE_THR_MM               800     // front ToF < 0.8m => hold
#define OBSTACLE_MAX_WAIT_MS          30000   // 30s blocked -> land
#define LOOP_DT_S                     (0.01f) // if autonavUpdate() called at 100 Hz
#define HOVER_THRUST_BASE             30000.0f// set for your frame after bench test

// Altitude PID (thrust control)
#define Z_KP 1.0f
#define Z_KI 0.04f
#define Z_KD 0.08f
// Trajectory timing
#define SEGMENT_TIME_MS 3000   // ms per edge (3s)
#define SHAPE_SPEED     0.2f   // m/s horizontal speed (tune!)

static uint64_t shapeStartUs = 0;
// ---- EXTERNAL SENSOR HOOKS ----
// Implement these in your sensor drivers or glue once and they’re reusable.
extern bool sensorsGetDownTofMm(uint16_t* out_mm);   // downward VL53L0X/L1X
extern bool sensorsGetFrontTofMm(uint16_t* out_mm);  // forward VL53L1X
// Optionally: extern void motorsShutDown(void);

// ---- STATE ----
static autonav_state_t s_state = AUTONAV_IDLE;
static uint16_t s_targetAltMm = AUTONAV_DEFAULT_ALT_MM;
static uint8_t  s_shapeId = 0;

static uint64_t s_lastCmdUs = 0;     // heartbeat updated by autonavKickSafety()
static uint64_t s_obstEnterUs = 0;   // when we entered HOLD_OBSTACLE
static float z_i = 0.f, z_prevErr = 0.f;
static float altFilt = AUTONAV_DEFAULT_ALT_MM;

// ---- utils ----
static inline uint64_t nowUs(void){ return (uint64_t)esp_timer_get_time(); }
static inline uint64_t msSince(uint64_t t0){ return (nowUs() - t0) / 1000ULL; }

void autonavSetTargetAltMm(uint16_t mm){ s_targetAltMm = mm; }
autonav_state_t autonavGetState(void){ return s_state; }

void autonavKickSafety(void){ s_lastCmdUs = nowUs(); }
void autonavEnterOverride(void) { s_state = AUTONAV_OVERRIDE; }
void autonavExitOverride(void)  { s_lastCmdUs = nowUs(); s_state = AUTONAV_RUNNING; }
bool autonavIsOverride(void)    { return s_state == AUTONAV_OVERRIDE; }

void autonavInit(void){
  autonav_crtp_start();
  s_state = AUTONAV_IDLE;
  s_targetAltMm = AUTONAV_DEFAULT_ALT_MM;
  s_shapeId = 0;
  s_lastCmdUs = nowUs();
  s_obstEnterUs = 0;
  z_i = 0.f; z_prevErr = 0.f; altFilt = (float)s_targetAltMm;
}

void autonavStartShape(uint8_t shapeId){
    s_shapeId = shapeId;
    s_state = AUTONAV_RUNNING;
    shapeStartUs = nowUs();
    autonavKickSafety();
}

void autonavStop(void){
  s_shapeId = 0;
  s_state = AUTONAV_IDLE;
}

// Simple thrust controller to hold altitude at s_targetAltMm using down ToF
static uint16_t zHoldThrustCmd(bool* tofOkOut){
  uint16_t altMm;
  bool ok = sensorsGetDownTofMm(&altMm);
  if (tofOkOut) *tofOkOut = ok;

  if (!ok){
    // No valid ToF: gentle hover-ish thrust (don’t climb)
    return (uint16_t)(HOVER_THRUST_BASE * 0.93f);
  }

  // Low-pass filter ToF to reduce noise
  altFilt = 0.9f * altFilt + 0.1f * (float)altMm;

  float err = ((float)s_targetAltMm) - altFilt;
  z_i += err * LOOP_DT_S;
  if (z_i > 400.f) z_i = 400.f;      // anti-windup
  if (z_i < -400.f) z_i = -400.f;
  float z_d = (err - z_prevErr) / LOOP_DT_S;
  z_prevErr = err;

  float thrust = HOVER_THRUST_BASE + (Z_KP*err + Z_KI*z_i + Z_KD*z_d);

  if (thrust < 20000.f) thrust = 20000.f;
  if (thrust > 65000.f) thrust = 65000.f;

  return (uint16_t)thrust;
}

// Land command: ramp down thrust safely
static void commandLand(setpoint_t* sp){
  memset(sp, 0, sizeof(*sp));
  sp->thrust = 0; // If you have a real land sequence, call it instead.
  s_state = AUTONAV_LANDING;
}

void autonavUpdate(uint32_t tickMs){
  // 1) 30s safety timeout: if no app heartbeat, land.
  if (msSince(s_lastCmdUs) > SAFETY_TIMEOUT_MS && s_state != AUTONAV_LANDING && s_state != AUTONAV_LANDED){
    s_state = AUTONAV_LANDING;
  }

  // 2) Build setpoint
  setpoint_t sp; memset(&sp, 0, sizeof(sp));

  // Thrust for Z-hold (or idle/land)
  bool zOk = false;
  uint16_t thrust = zHoldThrustCmd(&zOk);

  // 3) Obstacle logic (front ToF)
  uint16_t frontMm = 0xFFFF;
  bool frontOk = sensorsGetFrontTofMm(&frontMm);
  bool blocked = (frontOk && frontMm < OBSTACLE_THR_MM);
  if (s_state == AUTONAV_OVERRIDE) {
    // Manual override: don't generate autonomous setpoints
    autonavKickSafety();   // keep safety timer alive
    return;
}
  switch (s_state){
    case AUTONAV_IDLE:
      // Keep it idle on ground (no thrust), or very low to avoid spin-ups.
      sp.thrust = 0;
      break;

    case AUTONAV_RUNNING:
    if (blocked){
        s_state = AUTONAV_HOLD_OBSTACLE;
        s_obstEnterUs = nowUs();
    } else {
        sp.thrust = thrust;   // keep altitude at 1.2 m

        // --- Shape logic ---
        uint32_t tMs = msSince(shapeStartUs);
        uint32_t seg = (tMs / SEGMENT_TIME_MS);

        switch(s_shapeId){
            case 1: // Square
                seg %= 4;
                if (seg == 0) { sp.velocity.x = SHAPE_SPEED; sp.velocity.y = 0; }
                if (seg == 1) { sp.velocity.x = 0; sp.velocity.y = SHAPE_SPEED; }
                if (seg == 2) { sp.velocity.x = -SHAPE_SPEED; sp.velocity.y = 0; }
                if (seg == 3) { sp.velocity.x = 0; sp.velocity.y = -SHAPE_SPEED; }
                break;

            case 2: // Rectangle
                seg %= 4;
                if (seg == 0) { sp.velocity.x = SHAPE_SPEED; sp.velocity.y = 0; }
                if (seg == 1) { sp.velocity.x = 0; sp.velocity.y = SHAPE_SPEED/2; }
                if (seg == 2) { sp.velocity.x = -SHAPE_SPEED; sp.velocity.y = 0; }
                if (seg == 3) { sp.velocity.x = 0; sp.velocity.y = -SHAPE_SPEED/2; }
                break;

            case 3: // Triangle
                seg %= 3;
                if (seg == 0) { sp.velocity.x = SHAPE_SPEED; sp.velocity.y = 0; }
                if (seg == 1) { sp.velocity.x = -SHAPE_SPEED/2; sp.velocity.y = SHAPE_SPEED*0.87f; }
                if (seg == 2) { sp.velocity.x = -SHAPE_SPEED/2; sp.velocity.y = -SHAPE_SPEED*0.87f; }
                break;

            case 4: // Oval (approximate with sine wave)
                sp.velocity.x = SHAPE_SPEED * cosf(tMs/1000.0f);
                sp.velocity.y = (SHAPE_SPEED/2) * sinf(tMs/1000.0f);
                break;
	     case 5: // Pentagon
                seg %= 5;
                {
                    float angle = seg * (2.0f * M_PI / 5.0f);  // 72° per side
                    sp.velocity.x = SHAPE_SPEED * cosf(angle);
                    sp.velocity.y = SHAPE_SPEED * sinf(angle);
                }
                break;
        }
    }
    break;
    case AUTONAV_HOLD_OBSTACLE:
      // Stop XY, hold Z = 1.2m
      sp.thrust = thrust;

      if (!blocked){
        // Obstacle cleared -> resume path
        s_state = AUTONAV_RUNNING;
      } else if (msSince(s_obstEnterUs) > OBSTACLE_MAX_WAIT_MS){
        // Blocked too long -> land
        s_state = AUTONAV_LANDING;
      }
      break;

    case AUTONAV_LANDING:
      commandLand(&sp);
      // After sending zero thrust once, mark as landed.
      s_state = AUTONAV_LANDED;
      break;

    case AUTONAV_LANDED:
      sp.thrust = 0;
      break;
  case AUTONAV_OVERRIDE:
    // Manual override: don’t run auto-nav logic, let commander take over.
    break;
  }

  // Send setpoint (roll/pitch/yaw zeroed; you’ll add XY later)
  commanderSetSetpoint(&sp, tickMs);
}


