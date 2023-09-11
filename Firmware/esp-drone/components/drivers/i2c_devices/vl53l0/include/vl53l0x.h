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
 * vl53l0x.h: Time-of-flight distance sensor driver
 */

#ifndef _VL53L0X_H_
#define _VL53L0X_H_

#include "i2cdev.h"

#define VL53L0X_DEFAULT_ADDRESS 0x29  //0b0101001
#define VL53L0X_ID			0xEEAA

#define VL53L0X_RA_SYSRANGE_START                              0x00

#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                   0x8a

#define VL53L0X_RA_SYSTEM_THRESH_HIGH                          0x0C
#define VL53L0X_RA_SYSTEM_THRESH_LOW                           0x0E

#define VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG                      0x01
#define VL53L0X_RA_SYSTEM_RANGE_CONFIG                         0x09
#define VL53L0X_RA_SYSTEM_INTERMEASUREMENT_PERIOD              0x04

#define VL53L0X_RA_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A

#define VL53L0X_RA_GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR                      0x0B

#define VL53L0X_RA_RESULT_INTERRUPT_STATUS                     0x13
#define VL53L0X_RA_RESULT_RANGE_STATUS                         0x14

#define VL53L0X_RA_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define VL53L0X_RA_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define VL53L0X_RA_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define VL53L0X_RA_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define VL53L0X_RA_RESULT_PEAK_SIGNAL_RATE_REF                 0xB6

#define VL53L0X_RA_ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28

#define VL53L0X_RA_I2C_SLAVE_DEVICE_ADDRESS                    0x8A

#define VL53L0X_RA_MSRC_CONFIG_CONTROL                         0x60

#define VL53L0X_RA_PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define VL53L0X_RA_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64

#define VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

#define VL53L0X_RA_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define VL53L0X_RA_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62

#define VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52

#define VL53L0X_RA_SYSTEM_HISTOGRAM_BIN                        0x81
#define VL53L0X_RA_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define VL53L0X_RA_HISTOGRAM_CONFIG_READOUT_CTRL               0x55

#define VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define VL53L0X_RA_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP                  0x46

#define VL53L0X_RA_SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define VL53L0X_RA_IDENTIFICATION_MODEL_ID                     0xC0
#define VL53L0X_RA_IDENTIFICATION_REVISION_ID                  0xC2

#define VL53L0X_IDENTIFICATION_MODEL_ID                        0xEEAA
#define VL53L0X_IDENTIFICATION_REVISION_ID                     0x10

#define VL53L0X_RA_OSC_CALIBRATE_VAL                           0xF8

#define VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5

#define VL53L0X_RA_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define VL53L0X_RA_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define VL53L0X_RA_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VL53L0X_RA_POWER_MANAGEMENT_GO1_POWER_FORCE            0x80

#define VL53L0X_RA_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

#define VL53L0X_RA_ALGO_PHASECAL_LIM                           0x30
#define VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
typedef struct {
  bool tcc;
  bool msrc;
  bool dss;
  bool pre_range;
  bool final_range;
} SequenceStepEnables;

typedef struct
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

typedef enum vcselPeriodType_t { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

typedef struct
{
  uint8_t devAddr;
  I2C_Dev *I2Cx;

  uint16_t io_timeout;
  bool did_timeout;
  uint16_t timeout_start_ms;

  // read by init and used when starting measurement;
  // is StopVariable field of VL53L0X_DevData_t structure in API
  uint8_t stop_variable;

  uint32_t measurement_timing_budget_us;
  uint16_t measurement_timing_budget_ms;
} VL53L0xDev;

/** Default constructor, uses external I2C address.
 * @see VL53L0X_DEFAULT_ADDRESS
 */
bool vl53l0xInit(VL53L0xDev* dev, I2C_Dev *I2Cx, bool io_2V8);

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool vl53l0xTestConnection(VL53L0xDev* dev);

/** Get Model ID.
 * This register is used to verify the model number of the device,
 * but only before it has been configured to run
 * @return Model ID
 * @see VL53L0X_RA_IDENTIFICATION_MODEL_ID
 * @see VL53L0X_IDENTIFICATION_MODEL_ID
 */
uint16_t vl53l0xGetModelID(VL53L0xDev* dev);

/** Get Revision ID.
 * This register is used to verify the revision number of the device,
 * but only before it has been configured to run
 * @return Revision ID
 * @see VL53L0X_RA_IDENTIFICATION_REVISION_ID
 * @see VL53L0X_IDENTIFICATION_REVISION_ID
 */
uint8_t vl53l0xGetRevisionID(VL53L0xDev* dev);

/** Set I2C address
 * Any subsequent communication will be on the new address
 * The address passed is the 7bit I2C address from LSB (ie. without the
 * read/write bit)
 */
bool vl53l0xSetI2CAddress(VL53L0xDev* dev, uint8_t address);

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool vl53l0xInitSensor(VL53L0xDev* dev, bool io_2v8);

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool vl53l0xSetSignalRateLimit(VL53L0xDev* dev, float limit_Mcps);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool vl53l0xSetMeasurementTimingBudget(VL53L0xDev* dev, uint32_t budget_us);

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t vl53l0xGetMeasurementTimingBudget(VL53L0xDev* dev);

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool vl53l0xSetVcselPulsePeriod(VL53L0xDev* dev, vcselPeriodType type, uint8_t period_pclks);

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t vl53l0xGetVcselPulsePeriod(VL53L0xDev* dev, vcselPeriodType type);

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void vl53l0xStartContinuous(VL53L0xDev* dev, uint32_t period_ms);

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void vl53l0xStopContinuous(VL53L0xDev* dev);

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t vl53l0xReadRangeContinuousMillimeters(VL53L0xDev* dev);

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t vl53l0xReadRangeSingleMillimeters(VL53L0xDev* dev);

uint16_t vl53l0xReadReg16Bit(VL53L0xDev* dev, uint8_t reg);


#endif /* _VL53L0X_H_ */
