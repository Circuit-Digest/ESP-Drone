/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
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
 * @file eeprom.h
 * Driver for the 24AA64F eeprom. esp-drone use nvs
 *
 */
#ifndef EERROM_H
#define EERROM_H

#include <stdbool.h>
#include "i2cdev.h"

#define EEPROM_IN_FLASH_ADDR  0x3d2000 //TODO: CHANGE 20200106 NEED TO TEST
#define EEPROM_I2C_ADDR     0x50 //dont have a eeprom just for latency
#define EEPROM_SIZE         0x1FFF   //EEPROM_IN_FLASH_ADDR

/**
 * Initialize the i2c eeprom module
 * @param i2cPort  I2C port ( a CPAL_InitTypeDef) the eeprom is connected to.
 *
 * @return True on success, else false.
 */
bool eepromInit(I2C_Dev *i2cPort);

/**
 * Test that the eeprom is there
 *
 * @return True on success, else false.
 */
bool eepromTest(void);

/**
 * Test that the eeprom is there
 *
 * @return True on success, else false.
 */
bool eepromTestConnection(void);

/**
 * Read data from the eeprom into a supplied buffer
 * @param buffer  The buffer to write the data in
 * @param readAddr  The start address to read from
 * @param len  Number of bytes to read
 *
 * @return True on success, else false.
 */
bool eepromReadBuffer(uint8_t *buffer, size_t readAddr, size_t len);

/**
 * Write data to the eeprom from a supplied buffer.
 * Currently very slow as id does single byte write.
 * @param buffer  The buffer to read the data from
 * @param writeAddr  The start address to write to
 * @param len  Number of bytes to write
 *
 * @return True on success, else false.
 */
bool eepromWriteBuffer(uint8_t *buffer, size_t writeAddr, size_t len);

// TODO
bool eepromWritePage(uint8_t *buffer, uint16_t writeAddr);

#endif // EERROM_H
