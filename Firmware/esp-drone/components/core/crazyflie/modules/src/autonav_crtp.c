#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "autonav.h"
#include "autonav_crtp.h"

// --- Optional: use CRTP if available, otherwise compile to no-op ---
#if __has_include("crtp.h")
  #include "crtp.h"
  #define AUTONAV_HAVE_CRTP 1
#else
  #define AUTONAV_HAVE_CRTP 0
#endif

// If you have a down-ToF accessor, declare it so we can export status
extern bool sensorsGetDownTofMm(uint16_t* out_mm);

// Small helper
static inline uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

#if AUTONAV_HAVE_CRTP
static void autonav_handle_packet(const CRTPPacket* pk)
{
  if (!pk) return;
  if (pk->port != AUTONAV_CRTP_PORT || pk->channel != AUTONAV_CRTP_CH) return;
  if (pk->size < 1) return;

  const uint8_t cmd = pk->data[0];

  switch (cmd) {
    case AUTONAV_CMD_STOP:
      autonavStop();
      autonavKickSafety();
      break;

    case AUTONAV_CMD_SQUARE:
      autonavStartShape(1);
      autonavKickSafety();
      break;

    case AUTONAV_CMD_RECT:
      autonavStartShape(2);
      autonavKickSafety();
      break;

    case AUTONAV_CMD_OVAL:
      autonavStartShape(3);
      autonavKickSafety();
      break;

    case AUTONAV_CMD_TRI:
      autonavStartShape(4);
      autonavKickSafety();
      break;

    case AUTONAV_CMD_SET_ALT_MM: {
      if (pk->size >= 3) {
        uint16_t mm = u16le(&pk->data[1]);
        autonavSetTargetAltMm(mm);
        autonavKickSafety();
      }
      break;
    }

    case AUTONAV_CMD_OVERRIDE_ON:
      autonavEnterOverride();
      autonavKickSafety();
      break;

    case AUTONAV_CMD_OVERRIDE_OFF:
      // Resume RUNNING if a shape was selected, else go IDLE
      autonavKickSafety();
      break;

    default:
      break;
  }

  // Optional: send back a tiny status frame (same port/ch)
  CRTPPacket out = {0};
  out.port = AUTONAV_CRTP_PORT;
  out.channel = AUTONAV_CRTP_CH;
  out.size = sizeof(autonav_status_t);
  autonav_status_t s = {0};
  s.state = (uint8_t)autonavGetState();
  uint16_t alt = 0;
  (void)sensorsGetDownTofMm(&alt);
  s.altMm = alt;

  memcpy(out.data, &s, sizeof(s));
  crtpSendPacketBlock(&out);
}
#endif // AUTONAV_HAVE_CRTP

static void autonav_crtp_task(void *arg)
{
#if AUTONAV_HAVE_CRTP
  // Passive receiver loop: pull packets and parse ours
  for (;;) {
    CRTPPacket pk = {0};
    if (crtpReceivePacketBlock(&pk) == 0) {
      autonav_handle_packet(&pk);
    }
  }
#else
  // No CRTP on this platform build: keep task alive, do nothing.
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
#endif
}

void autonav_crtp_start(void)
{
  static bool started = false;
  if (started) return;
  started = true;
  xTaskCreate(autonav_crtp_task, "autonav_crtp", 2048, NULL, tskIDLE_PRIORITY+2, NULL);
}
