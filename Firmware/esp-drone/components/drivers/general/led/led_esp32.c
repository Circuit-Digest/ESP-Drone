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
 * led.c - LED handing functions
 */
#include <stdbool.h>

/*FreeRtos includes*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led.h"
#include "stm32_legacy.h"

static unsigned int led_pin[] = {
    [LED_BLUE] = LED_GPIO_BLUE,
    [LED_RED]   = LED_GPIO_RED,
    [LED_GREEN] = LED_GPIO_GREEN,
};
static int led_polarity[] = {
    [LED_BLUE] = LED_POL_BLUE,
    [LED_RED]   = LED_POL_RED,
    [LED_GREEN] = LED_POL_GREEN,
};

static bool isInit = false;

//Initialize the green led pin as output
void ledInit()
{
    int i;

    if (isInit) {
        return;
    }

    for (i = 0; i < LED_NUM; i++) {
        gpio_config_t io_conf = {
            //bit mask of the pins that you want to set,e.g.GPIO18/19
            .pin_bit_mask = (1ULL << led_pin[i]),
            //disable pull-down mode
            .pull_down_en = 0,
            //disable pull-up mode
            .pull_up_en = 0,
            //set as output mode
            .mode = GPIO_MODE_OUTPUT,
        };
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        ledSet(i, 0);
    }

    isInit = true;
}

bool ledTest(void)
{
    ledSet(LED_GREEN, 1);
    ledSet(LED_RED, 0);
    vTaskDelay(M2T(250));
    ledSet(LED_GREEN, 0);
    ledSet(LED_RED, 1);
    vTaskDelay(M2T(250));
    // LED test end
    ledClearAll();
    ledSet(LED_BLUE, 1);

    return isInit;
}

void ledClearAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn off the LED:s
        ledSet(i, 0);
    }
}

void ledSetAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn on the LED:s
        ledSet(i, 1);
    }
}
void ledSet(led_t led, bool value)
{
    if (led > LED_NUM || led == LED_NUM) {
        return;
    }

    if (led_polarity[led] == LED_POL_NEG) {
        value = !value;
    }

    if (value) {
        gpio_set_level(led_pin[led], 1);
    } else {
        gpio_set_level(led_pin[led], 0);
    }
}


