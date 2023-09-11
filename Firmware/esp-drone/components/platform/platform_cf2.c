/**
*
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * Platform functionality for the different hardware
 */



#include <string.h>

#include "platform.h"
#define DEBUG_MODULE "PLATFORM"
#include "debug_cf.h"

/*to support different hardware platform */
static platformConfig_t configs[] = {

    {
        .deviceType = "EP20",
        .deviceTypeName = "ESPlane 2.0 ",
        .sensorImplementation = SensorImplementation_mpu6050_HMC5883L_MS5611,
        .physicalLayoutAntennasAreClose = false,
        .motorMap = motorMapDefaultBrushed,
    },
    {
        .deviceType = "ED12",
        .deviceTypeName = "ESP_Drone_v1_2",
        .sensorImplementation = SensorImplementation_mpu6050_HMC5883L_MS5611,
        .physicalLayoutAntennasAreClose = false,
        .motorMap = motorMapDefaultBrushed,
    },

};

const platformConfig_t *platformGetListOfConfigurations(int *nrOfConfigs)
{
    *nrOfConfigs = sizeof(configs) / sizeof(platformConfig_t);
    return configs;
}

bool platformInitHardware()
{
    //TODO:
    return true;
}

// Config functions ------------------------

const char *platformConfigGetPlatformName()
{
    return "ED12";
}
