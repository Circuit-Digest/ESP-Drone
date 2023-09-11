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
 * piezo.c - Piezo/Buzzer driver
 *
 * This code mainly interfacing the PWM peripheral lib of ESP32.
 */

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#include "stm32_legacy.h"
#include "piezo.h"

// HW defines
#define PIEZO_TIM_PERIF       RCC_APB1Periph_TIM5
#define PIEZO_TIM             TIM5
#define PIEZO_TIM_DBG         DBGMCU_TIM2_STOP
#define PIEZO_TIM_SETCOMPARE  TIM_SetCompare2
#define PIEZO_TIM_GETCAPTURE  TIM_GetCapture2

#define BUZ_PWM_CH1  0
#define BUZ_PWM_CH2  1

#define PIEZO_GPIO_POS_PIN    CONFIG_BUZ1_PIN_POS // buzzer+ -> GPIO39 ; buzzer- -> GND:
#define PIEZO_GPIO_NEG_PIN    CONFIG_BUZ2_PIN_NEG // GND

#define PIEZO_PWM_BITS      (13)
#define PIEZO_PWM_PERIOD    ((1<<PIEZO_PWM_BITS) - 1)
#define PIEZO_PWM_PRESCALE  (0)

/* This should be calculated.. */
#define PIEZO_BASE_FREQ (329500)

static bool isInit = false;

ledc_channel_config_t buzz_channel[1] = {
    {
        .channel = BUZ_PWM_CH1,
        .duty = 0,
        .gpio_num = PIEZO_GPIO_POS_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1

    },
};

/* Public functions */

void piezoInit()
{
    if (isInit) {
        return;
    }

    //Clock the gpio and the timers
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PIEZO_PWM_BITS, // resolution of PWM duty
        .freq_hz = 4000,                     // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_1,            // timer index
        //.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {

    }

    for (uint8_t i = 0; i < 1; i++) {
        ledc_channel_config(&buzz_channel[i]);
    }

    isInit = true;
}

bool piezoTest(void)
{
    return isInit;
}

void piezoSetRatio(uint8_t ratio)
{
    uint16_t ratio16 = 0;
    if (ratio > 0) {
        ratio16 = ratio << 5;
    }
    ledc_set_duty(buzz_channel[0].speed_mode, buzz_channel[0].channel, ratio16);
    ledc_update_duty(buzz_channel[0].speed_mode, buzz_channel[0].channel);

}

void piezoSetFreq(uint16_t freq)
{
    if ( freq <= 0 ) {
        piezoSetRatio(0);
    } else {
        ledc_set_freq(buzz_channel[0].speed_mode, buzz_channel[0].timer_sel, freq);
    }
}
