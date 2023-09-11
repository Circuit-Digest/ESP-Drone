/*
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
 * deck_spi.c - Deck-API SPI communication implementation
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"

#include "deck_spi.h"
#include "config.h"
#include "cfassert.h"
#include "nvicconf.h"
#define DEBUG_MODULE "DECK_SPI"
#include "debug_cf.h"

#ifdef CONFIG_EXT_FLOW_TESTBOARD //This board use wrong pin definition,only for test.
    #define SPI_SCK_PIN CONFIG_SPI_PIN_MOSI
    #define SPI_MOSI_PIN CONFIG_SPI_PIN_CLK
#else
    #define SPI_SCK_PIN CONFIG_SPI_PIN_CLK
    #define SPI_MOSI_PIN CONFIG_SPI_PIN_MOSI
#endif
#define SPI_MISO_PIN CONFIG_SPI_PIN_MISO

#define DUMMY_BYTE 0xA5

static bool isInit = false;
static SemaphoreHandle_t spiMutex;

static void spiConfigureWithSpeed(uint32_t baudRatePrescaler);

static spi_device_handle_t spi;

void spiBegin(void)
{

    if (isInit) {
        return;
    }

    spiMutex = xSemaphoreCreateMutex();

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    }; //Defaults to 4094 if 0
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_BAUDRATE_2MHZ, //Clock out at 10 MHz
        .mode = 3,							 //SPI mode 0
        .spics_io_num = -1,					 //CS pin
        .queue_size = 8,					 //We want to be able to queue 7 transactions at a time
        /*.pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line*/
    };
    //Initialize the SPI bus
    spi_host_device_t host_id = SPI2_HOST;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0))
    ret = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
#else
    int dma_chan = host_id; //set dma channel equals to host_id by default
    ret = spi_bus_initialize(host_id, &buscfg, dma_chan);
#endif
    ESP_ERROR_CHECK(ret);
    //Attach the pmw3901 to the SPI bus
    ret = spi_bus_add_device(host_id, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    isInit = true;
}

static void spiConfigureWithSpeed(uint32_t baudRatePrescaler)
{
    //TODO:
}

bool spiTest(void)
{
    return isInit;
}

bool spiExchange(size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx)
{
    if (isInit != true) {
        return false;
    }

    if (length == 0) {
        return true;    //no need to send anything
    }

    esp_err_t ret;

    if (is_tx == true) {

        static spi_transaction_t t;
        memset(&t, 0, sizeof(t));					//Zero out the transaction
        t.length = length * 8;						//Len is in bytes, transaction length is in bits.
        t.tx_buffer = data_tx;						//Data
        ret = spi_device_polling_transmit(spi, &t); //Transmit!
        assert(ret == ESP_OK);						//Should have had no issues.
        //DEBUG_PRINTD("spi send = %d",t.length);
        return true;
    }

    static spi_transaction_t r;
    memset(&r, 0, sizeof(r));
    r.length = length * 8;
    r.flags = SPI_TRANS_USE_RXDATA;
    ret = spi_device_polling_transmit(spi, &r);
    assert(ret == ESP_OK);

    if (r.rxlength > 0) {
        //DEBUG_PRINTD("rxlength = %d",r.rxlength);
        memcpy(data_rx, r.rx_data, length);
    }

    return true;
}

void spiBeginTransaction(uint32_t baudRatePrescaler)
{
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction()
{
    xSemaphoreGive(spiMutex);
}
