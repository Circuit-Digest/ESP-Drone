/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone firmware
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
 * pm_esplane.h - Power Management driver and functions.
 */

#ifndef PM_H_
#define PM_H_

#include "esp_idf_version.h"
#include "syslink.h"
#include "driver/adc.h"
//#include "deck.h"

#ifndef CRITICAL_LOW_VOLTAGE
  #define PM_BAT_CRITICAL_LOW_VOLTAGE   3.0f
#else
  #define PM_BAT_CRITICAL_LOW_VOLTAGE   CRITICAL_LOW_VOLTAGE
#endif
#ifndef CRITICAL_LOW_TIMEOUT
  #define PM_BAT_CRITICAL_LOW_TIMEOUT   M2T(1000 * 5) // 5 sec default
#else
  #define PM_BAT_CRITICAL_LOW_TIMEOUT   CRITICAL_LOW_TIMEOUT
#endif

#ifndef LOW_VOLTAGE
  #define PM_BAT_LOW_VOLTAGE   3.2f
#else
  #define PM_BAT_LOW_VOLTAGE   LOW_VOLTAGE
#endif
#ifndef LOW_TIMEOUT
  #define PM_BAT_LOW_TIMEOUT   M2T(1000 * 5) // 5 sec default
#else
  #define PM_BAT_LOW_TIMEOUT   LOW_TIMEOUT
#endif

#ifndef SYSTEM_SHUTDOWN_TIMEOUT
  #define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * 5) // 5 min default
#else
  #define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * SYSTEM_SHUTDOWN_TIMEOUT)
#endif

#define PM_BAT_DIVIDER                3.0f
#define PM_BAT_ADC_FOR_3_VOLT         (int32_t)(((3.0f / PM_BAT_DIVIDER) / 2.8f) * 4096)
#define PM_BAT_ADC_FOR_1p2_VOLT       (int32_t)(((1.2f / PM_BAT_DIVIDER) / 2.8f) * 4096)

#define PM_BAT_IIR_SHIFT     8
/**
 * Set PM_BAT_WANTED_LPF_CUTOFF_HZ to the wanted cut-off freq in Hz.
 */
#define PM_BAT_WANTED_LPF_CUTOFF_HZ   1

/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation.
 * attenuation = fs / 2*pi*f0
 */
#define PM_BAT_IIR_LPF_ATTENUATION (int)(ADC_SAMPLING_FREQ / (int)(2 * 3.1415f * PM_BAT_WANTED_LPF_CUTOFF_HZ))
#define PM_BAT_IIR_LPF_ATT_FACTOR  (int)((1<<PM_BAT_IIR_SHIFT) / PM_BAT_IIR_LPF_ATTENUATION)

typedef enum
{
  battery,
  charging,
  charged,
  lowPower,
  shutDown,
} PMStates;

typedef enum
{
  charge100mA,
  charge300mA,
  charge500mA,
  chargeMax,
} PMChargeStates;

typedef enum
{
  USBNone,
  USB500mA,
  USBWallAdapter,
} PMUSBPower;

void pmInit(void);

bool pmTest(void);

/**
 * Power management task
 */
void pmTask(void *param);

void pmSetChargeState(PMChargeStates chgState);
void pmSyslinkUpdate(SyslinkPacket *slp);

/**
 * Returns the battery voltage i volts as a float
 */
float pmGetBatteryVoltage(void);

/**
 * Returns the min battery voltage i volts as a float
 */
float pmGetBatteryVoltageMin(void);

/**
 * Returns the max battery voltage i volts as a float
 */
float pmGetBatteryVoltageMax(void);

/**
 * Updates and calculates battery values.
 * Should be called for every new adcValues sample.
 */
//void pmBatteryUpdate(AdcGroup* adcValues);

/**
 * Returns true if the battery is below its low capacity threshold for an
 * extended period of time.
 */
bool pmIsBatteryLow(void);

/**
 * Returns true if the charger is currently connected
 */
bool pmIsChargerConnected(void);

/**
 * Returns true if the battery is currently charging
 */
bool pmIsCharging(void);

/**
 * Returns true if the battery is currently in use
 */
bool pmIsDischarging(void);

/**
 * Enable or disable external battery voltage measuring.
 */
void pmEnableExtBatteryVoltMeasuring(uint8_t pin, float multiplier);

/**
 * Measure an external voltage.
 */
float pmMeasureExtBatteryVoltage(void);

/**
 * Enable or disable external battery current measuring.
 */
void pmEnableExtBatteryCurrMeasuring(uint8_t pin, float ampPerVolt);

/**
 * Measure an external current.
 */
float pmMeasureExtBatteryCurrent(void);

#endif /* PM_H_ */
