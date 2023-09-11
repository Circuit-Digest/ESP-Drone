/**
 *
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2012 BitCraze AB
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
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//#include "deck.h"
#include "system.h"
#include "debug_cf.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "config.h"
#include "i2cdev.h"
#include "zranger.h"
#include "vl53l0x.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "cf_math.h"

// Measurement noise model
static const float expPointA = 1.0f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 1.3f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;

static VL53L0xDev dev;

static uint8_t vl53l0dataReady = 0;

static int16_t range1;
static int16_t dataready = 0;

int16_t zRangerMeasurementAndRestart()
{
	if (isInit == false)
	{
		return -1;
	}
  uint8_t val = 0;
	i2cdevReadByte(dev.I2Cx, dev.devAddr, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &val);

  if ((val & 0x07) == 0)
  {
    vl53l0dataReady = 0;

  }else
  {
    vl53l0dataReady = 1;
  }
    

	if (vl53l0dataReady != 0)
	{

   // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t rangetemp = vl53l0xReadReg16Bit(&dev, VL53L0X_RA_RESULT_RANGE_STATUS + 10);

		if (rangetemp != 0)
		{
			range1 = rangetemp;
			dataready = 1;
		}
		else
		{
			return -2;
		}

		i2cdevWriteByte(dev.I2Cx, dev.devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);; //clear Interrupt start next measurement
	}
	else
	{
		return -3;
	}
	return 0;
}

static int16_t zRangerGetMeasurementAndRestart(VL53L0xDev *dev)
{

	int16_t range;
	while (dataready == 0)
	{
		vTaskDelay(M2T(20));
	}
	dataready = 0;
	return range = range1;
}

void zRangerInit(void)
{
  if (isInit)
    return;

  dev.I2Cx = 0;
  dev.devAddr = VL53L0X_DEFAULT_ADDRESS;
  i2cdevInit(dev.I2Cx);

  dev.io_timeout = 0;
  dev.did_timeout = 0;
  dev.timeout_start_ms = 0;
  dev.stop_variable = 0;
  dev.measurement_timing_budget_us = 0;
  dev.measurement_timing_budget_ms = 0;

  vTaskDelay(M2T(20));

  uint16_t wordData;
  wordData = vl53l0xGetModelID(&dev);
  DEBUG_PRINTI( "VL53L0X: %02X\n\r", wordData);

  if(wordData == VL53L0X_ID)
	{
    DEBUG_PRINT( "VL53L0X I2C commection [OK].\n");
  }

  vl53l0xInit(&dev, I2C1_DEV, true);

  xTaskCreate(zRangerTask, ZRANGER_TASK_NAME, ZRANGER_TASK_STACKSIZE, NULL, ZRANGER_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRangerTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus  = vl53l0xTestConnection(&dev);

  return testStatus;
}

void zRangerTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodPreRange, 18);/*长距离模式  33ms 周期*/
  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodFinalRange, 14);/*长距离模式  33ms 周期*/
  vl53l0xStartContinuous(&dev, 0);
  
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(dev.measurement_timing_budget_ms));

    range_last = zRangerGetMeasurementAndRestart(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);
    DEBUG_PRINTD("ZRANGE = %f",range_last/ 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT) {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

bool zRangerReadRange(zDistance_t* zrange, const uint32_t tick)
{
  bool updated = false;

  if (isInit) {
    if (range_last != 0 && range_last < RANGE_OUTLIER_LIMIT) {
      zrange->distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      zrange->timestamp = tick;
      updated = true;
    }
  }
  return updated;
}

// static const DeckDriver zranger_deck = {
//   .vid = 0xBC,
//   .pid = 0x09,
//   .name = "bcZRanger",
//   .usedGpio = 0x0C,

//   .init = zRangerInit,
//   .test = zRangerTest,
// };

// DECK_DRIVER(zranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)
PARAM_GROUP_STOP(deck)
