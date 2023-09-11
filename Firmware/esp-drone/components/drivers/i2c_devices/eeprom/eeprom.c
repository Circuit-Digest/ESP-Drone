/**
 * ESP-Drone Firmware
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
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
 * @file eeprom.c
 * 使用 flash 分区代替 eeprom 保存数据，该部分需要检验，或替换成  NVS
 *
 */
#define DEBUG_MODULE "EEPROM"

#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "eeprom.h"
#include "debug_cf.h"
#include "eprintf.h"

#ifdef EEPROM_RUN_WRITE_READ_TEST
#include "esp_spi_flash.h"
static bool eepromTestWriteRead(void);
#endif

static bool isInit;

bool eepromInit(I2C_Dev *i2cPort)
{
    if (isInit) {
        return true;
    }

    //TODOspi_flash_init();
    DEBUG_PRINTI("spi_flash_init ... !");

    isInit = true;

    return true;
}

bool eepromTest(void)
{
    bool status;

    status = true;
    return status;
}

#ifdef EEPROM_RUN_WRITE_READ_TEST
static bool eepromTestWriteRead(void)
{
    bool status = true;
    int i;
    const uint8_t testData[20] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    uint8_t readData[20];

    for (i = 0; i < sizeof(testData) && status; i++) {
        // Write one byte with increasing addresses.
        eepromWriteBuffer(&testData[i], i, 1);
        // Read it all back and check
        eepromReadBuffer(readData, 0, i + 1);
        status = (memcmp(testData, readData, i + 1) == 0);
    }

    // Write it all.
    eepromWriteBuffer(testData, 0, sizeof(testData));

    for (i = 0; i < sizeof(testData) && status; i++) {
        // Read one byte with increasing addresses and check
        eepromReadBuffer(&readData[i], i, 1);
        status = (memcmp(testData, readData, i + 1) == 0);
    }

    return status;
}
#endif

bool eepromTestConnection(void)
{
    //uint8_t tmp;
    bool status;

    status = true;
    return status;
}

bool eepromReadBuffer(uint8_t *buffer, size_t readAddr, size_t len)
{
    bool status = false;
    esp_err_t err = ESP_OK; //TODO spi_flash_read(readAddr, buffer, len);

    if (err == ESP_OK) {

        status = true;
        DEBUG_PRINTI("spi_flash_read ok !");

    } else {
        DEBUG_PRINTW("spi_flash_read err = %d", err);
    }

    return status;
}

bool eepromWriteBuffer(uint8_t *buffer, size_t writeAddr, size_t len)
{
    bool status = false;
    writeAddr += EEPROM_IN_FLASH_ADDR;

    //TODO if (spi_flash_write(writeAddr, buffer, len) == ESP_OK) {
    if (1) {

        status = true;

    }

    return status;
}

bool eepromWritePage(uint8_t *buffer, uint16_t writeAddr)
{
//TODO: implement
    return false;
}
