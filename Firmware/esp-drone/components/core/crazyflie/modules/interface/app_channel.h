/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2020, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* app_channel.h: App realtime communication channel with the ground */
#pragma once

#include <stddef.h>
#include <stdbool.h>

#include "crtp.h"

#define APPCHANNEL_WAIT_FOREVER (-1)
#define APPCHANNEL_MTU (31)

/**
 * Send an app-channel packet
 * 
 * The maximum buffer size that can be sent is define in APPCHANNEL_MTU.
 * If the length of the buffer is longer than that, the packet will be cropped
 * to send only the APPCHANNEL_MTU first bytes.
 * 
 * This function can block if there is no more space in the Crazyflie TX queue.
 * This is very unlikely to happen when CRTP is connected but can happen when the
 * connection is not active.
 * 
 * @param data Pointer to the data buffer to be sent
 * @param length Length of the data buffer to send
 * 
 * \app_api
 */
void appchannelSendPacket(void* data, size_t length);

/**
 * Receive an app-channel packet
 * 
 * If the data received is longer than max_length, the data will be silently cropped and only
 * the fist "max_length" bytes of the packet will be copied in the buffer.
 * 
 * The maximum length packet possible to be received is APPCHANNEL_MTU bytes long.
 * 
 * @param buffer Data buffer where the packet content will be copied
 * @param max_length Maximum length of the data to be received, ie. length of the data buffer
 * @param timeout_ms Time to wait for a packet in millisecond. A value of 0 will make the
 *                   function non blocking, only reporting a packet is one is already in the
 *                   receive queue. A value of APPCHANNEL_WAIT_FOREVER make the funciton block
 *                   infinitly until a packet is received.
 * @return 0 if no packet has been received. The data length of the packet received.
 */
size_t appchannelReceivePacket(void* buffer, size_t max_length, int timeout_ms);

/**
 * Returns if an overflow has occured in the receive queue
 * 
 * The app-channel received packets are put in a queue. It is expected that the app is
 * regularly calling appchannelReceivePacket() to get the packets from the receive queue.
 * If that is not the case, the queue can overflow and this function allows the app to know
 * about it. The overflow flag is being reset by this call.
 * 
 * @return true if an overflow has occured in the receive queue.
 */
bool appchannelHasOverflowOccured();


// Function declared bellow are private to the Crazyflie firmware and
// should not be called from an app

/**
 * 
 */
void appchannelInit();

/**
 * 
 */
void appchannelIncomingPacket(CRTPPacket *p);
