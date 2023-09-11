#ifndef WIFI_ESP32_H_
#define WIFI_ESP32_H_
#include <stdbool.h>
#include <stdint.h>

#define WIFI_RX_TX_PACKET_SIZE   (64)

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[WIFI_RX_TX_PACKET_SIZE];
} UDPPacket;

/**
 * Initialize the wifi.
 *
 * @note Initialize CRTP link only if USE_CRTP_WIFI is defined
 */
void wifiInit(void);

/**
 * Test the WIFI status.
 *
 * @return true if the WIFI is initialized
 */
bool wifiTest(void);

/**
 * Get CRTP link data structure
 *
 * @return Address of the crtp link operations structure.
 */
//struct crtpLinkOperations * wifiGetLink();

/**
 * Get data from rx queue with timeout.
 * @param[out] c  Byte of data
 *
 * @return true if byte received, false if timout reached.
 */
bool wifiGetDataBlocking(UDPPacket *in);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If WIFI Crtp link is activated this function does nothing
 */
bool wifiSendData(uint32_t size, uint8_t* data);

#endif