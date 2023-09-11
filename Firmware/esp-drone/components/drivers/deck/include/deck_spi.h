/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2015 Bitcraze AB
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
 * deck_spi.h - Deck-API SPI communication header
 */
#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Based on 84MHz peripheral clock
#define SPI_BAUDRATE_21MHZ  21*1000*1000
#define SPI_BAUDRATE_12MHZ  12*1000*1000
#define SPI_BAUDRATE_6MHZ   6*1000*1000
#define SPI_BAUDRATE_3MHZ   3*1000*1000
#define SPI_BAUDRATE_2MHZ   2*1000*1000

/**
 * Initialize the SPI.
 */
void spiBegin(void);
void spiBeginTransaction(uint32_t baudRatePrescaler);
void spiEndTransaction();

/* Send the data_tx buffer and receive into the data_rx buffer */
bool spiExchange(size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx);

#endif /* SPI_H_ */
