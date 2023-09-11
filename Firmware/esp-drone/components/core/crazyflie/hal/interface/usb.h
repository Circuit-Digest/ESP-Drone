/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * usb.h - USB CRTP link and raw access functions
 */
#ifndef USB_H_
#define USB_H_

#include <stdbool.h>
#include <stdint.h>

#include "usbd_conf.h"


#define USB_RX_TX_PACKET_SIZE   (64)

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[USB_RX_TX_PACKET_SIZE];
} USBPacket;

/**
 * Initialize the UART.
 *
 * @note Initialize CRTP link only if USE_CRTP_UART is defined
 */
void usbInit(void);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool usbTest(void);

/**
 * Get CRTP link data structure
 *
 * @return Address of the crtp link operations structure.
 */
struct crtpLinkOperations * usbGetLink();

/**
 * Get data from rx queue with timeout.
 * @param[out] c  Byte of data
 *
 * @return true if byte received, false if timout reached.
 */
bool usbGetDataBlocking(USBPacket *in);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If UART Crtp link is activated this function does nothing
 */
bool usbSendData(uint32_t size, uint8_t* data);


#endif /* UART_H_ */
