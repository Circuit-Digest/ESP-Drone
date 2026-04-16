#pragma once
#include <stdint.h>

// CRTP port/channel we listen on
#define AUTONAV_CRTP_PORT   0x0D
#define AUTONAV_CRTP_CH     0

// Commands from app -> drone
typedef enum {
  AUTONAV_CMD_STOP        = 0,  // stop autonav (go IDLE)
  AUTONAV_CMD_SQUARE      = 1,
  AUTONAV_CMD_RECT        = 2,
  AUTONAV_CMD_OVAL        = 3,
  AUTONAV_CMD_TRI         = 4,
  AUTONAV_CMD_SET_ALT_MM  = 5,  // arg0: uint16 altitude (mm)
  AUTONAV_CMD_OVERRIDE_ON = 10, // manual override
  AUTONAV_CMD_OVERRIDE_OFF= 11  // resume autonav
} autonav_cmd_t;

// Minimal status the drone can send back (optional)
typedef struct __attribute__((packed)) {
  uint8_t state;   // autonav_state_t
  uint16_t altMm;  // latest down-ToF (mm) if available, else 0
} autonav_status_t;
