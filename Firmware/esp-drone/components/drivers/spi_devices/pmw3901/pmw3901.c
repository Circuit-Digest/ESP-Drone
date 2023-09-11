/*
 *
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
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
/* pwm3901.c: PMW3901 driver */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pmw3901.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "sleepus.h"
#include "deck_digital.h"
#include "deck_spi.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "PMW"
#include "debug_cf.h"

static bool isInit = false;

static void registerWrite(uint32_t csPin, uint8_t reg, uint8_t value)
{
    // Set MSB to 1 for write
    reg |= 0x80u;

    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(csPin, LOW);

    sleepus(50);

    spiExchange(1, 1, &reg, &reg);
    sleepus(50);
    spiExchange(1, 1, &value, &value);

    sleepus(50);

    digitalWrite(csPin, HIGH);
    spiEndTransaction();
    sleepus(200);
}

static uint8_t registerRead(uint32_t csPin, uint8_t reg)
{
    uint8_t data = 0;
    uint8_t dummy = 0;

    // Set MSB to 0 for read
    reg &= ~0x80u;

    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(csPin, LOW);

    sleepus(50);

    spiExchange(1, 1, &reg, &reg);
    sleepus(500);
    spiExchange(1, 0, &dummy, &data);

    sleepus(50);

    digitalWrite(csPin, HIGH);
    spiEndTransaction();
    sleepus(200);

    return data;
}

static void InitRegisters(uint32_t csPin)
{
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x61, 0xAD);
    registerWrite(csPin, 0x7F, 0x03);
    registerWrite(csPin, 0x40, 0x00);
    registerWrite(csPin, 0x7F, 0x05);
    registerWrite(csPin, 0x41, 0xB3);
    registerWrite(csPin, 0x43, 0xF1);
    registerWrite(csPin, 0x45, 0x14);
    registerWrite(csPin, 0x5B, 0x32);
    registerWrite(csPin, 0x5F, 0x34);
    registerWrite(csPin, 0x7B, 0x08);
    registerWrite(csPin, 0x7F, 0x06);
    registerWrite(csPin, 0x44, 0x1B);
    registerWrite(csPin, 0x40, 0xBF);
    registerWrite(csPin, 0x4E, 0x3F);
    registerWrite(csPin, 0x7F, 0x08);
    registerWrite(csPin, 0x65, 0x20);
    registerWrite(csPin, 0x6A, 0x18);
    registerWrite(csPin, 0x7F, 0x09);
    registerWrite(csPin, 0x4F, 0xAF);
    registerWrite(csPin, 0x5F, 0x40);
    registerWrite(csPin, 0x48, 0x80);
    registerWrite(csPin, 0x49, 0x80);
    registerWrite(csPin, 0x57, 0x77);
    registerWrite(csPin, 0x60, 0x78);
    registerWrite(csPin, 0x61, 0x78);
    registerWrite(csPin, 0x62, 0x08);
    registerWrite(csPin, 0x63, 0x50);
    registerWrite(csPin, 0x7F, 0x0A);
    registerWrite(csPin, 0x45, 0x60);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x4D, 0x11);
    registerWrite(csPin, 0x55, 0x80);
    registerWrite(csPin, 0x74, 0x1F);
    registerWrite(csPin, 0x75, 0x1F);
    registerWrite(csPin, 0x4A, 0x78);
    registerWrite(csPin, 0x4B, 0x78);
    registerWrite(csPin, 0x44, 0x08);
    registerWrite(csPin, 0x45, 0x50);
    registerWrite(csPin, 0x64, 0xFF);
    registerWrite(csPin, 0x65, 0x1F);
    registerWrite(csPin, 0x7F, 0x14);
    registerWrite(csPin, 0x65, 0x67);
    registerWrite(csPin, 0x66, 0x08);
    registerWrite(csPin, 0x63, 0x70);
    registerWrite(csPin, 0x7F, 0x15);
    registerWrite(csPin, 0x48, 0x48);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x41, 0x0D);
    registerWrite(csPin, 0x43, 0x14);
    registerWrite(csPin, 0x4B, 0x0E);
    registerWrite(csPin, 0x45, 0x0F);
    registerWrite(csPin, 0x44, 0x42);
    registerWrite(csPin, 0x4C, 0x80);
    registerWrite(csPin, 0x7F, 0x10);
    registerWrite(csPin, 0x5B, 0x02);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x40, 0x41);
    registerWrite(csPin, 0x70, 0x00);

    vTaskDelay(M2T(10)); // delay 10ms

    registerWrite(csPin, 0x32, 0x44);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x40, 0x40);
    registerWrite(csPin, 0x7F, 0x06);
    registerWrite(csPin, 0x62, 0xF0);
    registerWrite(csPin, 0x63, 0x00);
    registerWrite(csPin, 0x7F, 0x0D);
    registerWrite(csPin, 0x48, 0xC0);
    registerWrite(csPin, 0x6F, 0xD5);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x5B, 0xA0);
    registerWrite(csPin, 0x4E, 0xA8);
    registerWrite(csPin, 0x5A, 0x50);
    registerWrite(csPin, 0x40, 0x80);

    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x5A, 0x10);
    registerWrite(csPin, 0x54, 0x00);
}

bool pmw3901Init(uint32_t csPin)
{
    if (isInit) {
        return true;
    }

    // Initialize CS Pin
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    spiBegin();
    vTaskDelay(M2T(40));

    digitalWrite(csPin, HIGH);
    vTaskDelay(M2T(2));
    digitalWrite(csPin, LOW);
    vTaskDelay(M2T(2));
    digitalWrite(csPin, HIGH);
    vTaskDelay(M2T(2));

    uint8_t chipId    = registerRead(csPin, 0x00);
    uint8_t invChipId = registerRead(csPin, 0x5f);

    DEBUG_PRINTD("Motion chip id: 0x%x:0x%x\n", chipId, invChipId);

    if (chipId == 0x49 || invChipId == 0xB6) {
        // Power on reset
        registerWrite(csPin, 0x3a, 0x5a);
        vTaskDelay(M2T(5));

        // Reading the motion registers one time
        registerRead(csPin, 0x02);
        registerRead(csPin, 0x03);
        registerRead(csPin, 0x04);
        registerRead(csPin, 0x05);
        registerRead(csPin, 0x06);
        vTaskDelay(M2T(1));

        InitRegisters(csPin);

        isInit = true;
    }

    return isInit;
}

void pmw3901ReadMotion(uint32_t csPin, motionBurst_t *motion)
{
    uint8_t address = 0x16;

    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(csPin, LOW);
    sleepus(50);
    spiExchange(1, 1, &address, &address);
    sleepus(50);
    spiExchange(sizeof(motionBurst_t), 0, (uint8_t *)motion, (uint8_t *)motion);
    sleepus(50);
    digitalWrite(csPin, HIGH);
    spiEndTransaction();
    sleepus(50);

    uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
    realShutter |= (motion->shutter & 0x0ff) << 8;
    motion->shutter = realShutter;
}
