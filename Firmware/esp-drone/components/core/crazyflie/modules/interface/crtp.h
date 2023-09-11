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
 * crtp.h - CrazyRealtimeTransferProtocol stack
 */

#ifndef CRTP_H_
#define CRTP_H_

#include <stdint.h>
#include <stdbool.h>

#define CRTP_MAX_DATA_SIZE 30

#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))

#define CRTP_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)

typedef enum {
  CRTP_PORT_CONSOLE          = 0x00,
  CRTP_PORT_PARAM            = 0x02,
  CRTP_PORT_SETPOINT         = 0x03,
  CRTP_PORT_MEM              = 0x04,
  CRTP_PORT_LOG              = 0x05,
  CRTP_PORT_LOCALIZATION     = 0x06,
  CRTP_PORT_SETPOINT_GENERIC = 0x07,
  CRTP_PORT_SETPOINT_HL      = 0x08,
  CRTP_PORT_PLATFORM         = 0x0D,
  CRTP_PORT_LINK             = 0x0F,
} CRTPPort;

typedef struct _CRTPPacket
{
  uint8_t size;                         //< Size of data
  union {
    struct {
      union {
        uint8_t header;                 //< Header selecting channel and port
        struct {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel     : 2;      //< Selected channel within port
          uint8_t reserved    : 2;
          uint8_t port        : 4;      //< Selected port
#else
          uint8_t channel  : 2;
          uint8_t port     : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];  //< The full packet "raw"
  };
} __attribute__((packed)) CRTPPacket;

typedef void (*CrtpCallback)(CRTPPacket *);

/**
 * Initialize the CRTP stack
 */
void crtpInit(void);

bool crtpTest(void);

/**
 * Initializes the queue and dispatch of an task.
 *
 * @param[in] taskId The id of the CRTP task
 */
void crtpInitTaskQueue(CRTPPort taskId);

/**
 * Register a callback to be called for a particular port.
 *
 * @param[in] port Crtp port for which the callback is set
 * @param[in] cb Callback that will be called when a packet is received on
 *            'port'.
 *
 * @note Only one callback can be registered per port! The last callback
 *       registered will be the one called
 */
void crtpRegisterPortCB(int port, CrtpCallback cb);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the oldest lowest priority packet is dropped
 *
 * @param[in] p CRTPPacket to send
 */
int crtpSendPacket(CRTPPacket *p);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the function block until one place is free (Good for console implementation)
 */
int crtpSendPacketBlock(CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID.
 *
 * @param[in]  taskId The id of the CRTP task
 * @param[out] p      The CRTP Packet with infomation (unchanged if nothing to fetch)
 *
 * @returns status of fetch from queue
 */
int crtpReceivePacket(CRTPPort taskId, CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID. Wait some time befor giving up
 *
 * @param[in]  taskId The id of the CRTP task
 * @param[out] p      The CRTP Packet with infomation (unchanged if nothing to fetch)
 * @param[in] wait    Wait time in milisecond
 *
 * @returns status of fetch from queue
 */
int crtpReceivePacketWait(CRTPPort taskId, CRTPPacket *p, int wait);

/**
 * Get the number of free tx packets in the queue
 *
 * @return Number of free packets
 */
int crtpGetFreeTxQueuePackets(void);

/**
 * Wait for a packet to arrive for the specified taskID
 *
 * @param[in]  taskId The id of the CRTP task
 * @paran[out] p      The CRTP Packet with information
 *
 * @return status of fetch from queue
 */
int crtpReceivePacketBlock(CRTPPort taskId, CRTPPacket *p);

/**
 * Function pointer structure to be filled by the CRTP link to permits CRTP to
 * use manu link
 */
struct crtpLinkOperations
{
  int (*setEnable)(bool enable);
  int (*sendPacket)(CRTPPacket *pk);
  int (*receivePacket)(CRTPPacket *pk);
  bool (*isConnected)(void);
  int (*reset)(void);
};

void crtpSetLink(struct crtpLinkOperations * lk);

/**
 * Check if the connection timeout has been reached, otherwise
 * we will assume that we are connected.
 *
 * @return true if conencted, otherwise false
 */
bool crtpIsConnected(void);

/**
 * Reset the CRTP communication by flushing all the queues that
 * contain packages.
 *
 * @return 0 for success
 */
int crtpReset(void);

#endif /*CRTP_H_*/
