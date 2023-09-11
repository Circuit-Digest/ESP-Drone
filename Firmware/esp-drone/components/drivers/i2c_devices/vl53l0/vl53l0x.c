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

#include "debug_cf.h"

#include "i2cdev.h"
#include "vl53l0x.h"

// Record the current time to check an upcoming timeout against
#define startTimeout() (dev->timeout_start_ms = xTaskGetTickCount())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (dev->io_timeout > 0 && ((uint16_t)xTaskGetTickCount() - dev->timeout_start_ms) > dev->io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
static bool vl53l0xGetSpadInfo(VL53L0xDev *dev, uint8_t *count, bool *type_is_aperture);

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
static void vl53l0xGetSequenceStepEnables(VL53L0xDev *dev, SequenceStepEnables *enables);

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
static void vl53l0xGetSequenceStepTimeouts(VL53L0xDev *dev, SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts);

// based on VL53L0X_perform_single_ref_calibration()
static bool vl53l0xPerformSingleRefCalibration(VL53L0xDev *dev, uint8_t vhv_init_byte);

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t vl53l0xDecodeTimeout(uint16_t reg_val);

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks);

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

uint16_t vl53l0xReadReg16Bit(VL53L0xDev *dev, uint8_t reg);
static bool vl53l0xWriteReg16Bit(VL53L0xDev *dev, uint8_t reg, uint16_t val);
static bool vl53l0xWriteReg32Bit(VL53L0xDev *dev, uint8_t reg, uint32_t val);

//static int nextI2CAddress = VL53L0X_DEFAULT_ADDRESS + 1;

/** Default constructor, uses default I2C address.
 * @see VL53L0X_DEFAULT_ADDRESS
 */

bool vl53l0xInit(VL53L0xDev *dev, I2C_Dev *I2Cx, bool io_2V8)
{
    // dev->I2Cx = I2Cx;
    // dev->devAddr = VL53L0X_DEFAULT_ADDRESS;
    // i2cdevInit(dev->I2Cx);

    // dev->io_timeout = 0;
    // dev->did_timeout = 0;
    // dev->timeout_start_ms = 0;
    // dev->stop_variable = 0;
    // dev->measurement_timing_budget_us = 0;
    // dev->measurement_timing_budget_ms = 0;

    if (!vl53l0xInitSensor(dev, io_2V8)) {
        return false;
    }

    /* Move initialized sensor to a new I2C address */
    //int newAddress;

    // taskENTER_CRITICAL();
    // newAddress = nextI2CAddress++;
    // taskEXIT_CRITICAL();

    //return vl53l0xSetI2CAddress(dev, newAddress);
    return true;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool vl53l0xTestConnection(VL53L0xDev *dev)
{
    bool ret = true;
    ret &= vl53l0xGetModelID(dev) == VL53L0X_IDENTIFICATION_MODEL_ID;
    ret &= vl53l0xGetRevisionID(dev) == VL53L0X_IDENTIFICATION_REVISION_ID;
    return ret;
}

/** Get Model ID.
 * This register is used to verify the model number of the device,
 * but only before it has been configured to run
 * @return Model ID
 * @see VL53L0X_RA_IDENTIFICATION_MODEL_ID
 * @see VL53L0X_IDENTIFICATION_MODEL_ID
 */
uint16_t vl53l0xGetModelID(VL53L0xDev *dev)
{
    return vl53l0xReadReg16Bit(dev, VL53L0X_RA_IDENTIFICATION_MODEL_ID);
}

/** Get Revision ID.
 * This register is used to verify the revision number of the device,
 * but only before it has been configured to run
 * @return Revision ID
 * @see VL53L0X_RA_IDENTIFICATION_REVISION_ID
 * @see VL53L0X_IDENTIFICATION_REVISION_ID
 */
uint8_t vl53l0xGetRevisionID(VL53L0xDev *dev)
{
    uint8_t output = 0;
    i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_IDENTIFICATION_REVISION_ID, &output);
    return output;
}

/** Set I2C address
 * Any subsequent communication will be on the new address
 * The address passed is the 7bit I2C address from LSB (ie. without the
 * read/write bit)
 */
bool vl53l0xSetI2CAddress(VL53L0xDev *dev, uint8_t address)
{
    bool pass = i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, address);
    dev->devAddr = address;
    return pass;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool vl53l0xInitSensor(VL53L0xDev *dev, bool io_2v8)
{
    uint8_t temp;
    // VL53L0X_DataInit() begin

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (io_2v8) {
        i2cdevWriteBit(dev->I2Cx, dev->devAddr, VL53L0X_RA_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0, 0x01);
    }

    // "Set I2C standard mode"
    if (!i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x88, 0x00)) {
        return false;
    }

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x00);
    i2cdevReadByte(dev->I2Cx, dev->devAddr, 0x91, &dev->stop_variable);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_MSRC_CONFIG_CONTROL, &temp);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_MSRC_CONFIG_CONTROL, temp | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    vl53l0xSetSignalRateLimit(dev, 0.25);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin

    uint8_t spad_count;
    bool spad_type_is_aperture;

    if (!vl53l0xGetSpadInfo(dev, &spad_count, &spad_type_is_aperture)) {
        return false;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    i2cdevReadReg8(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    i2cdevWriteReg8(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x09, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x10, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x11, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x24, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x25, 0xFF);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x75, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x4E, 0x2C);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x48, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x30, 0x20);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x30, 0x09);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x54, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x31, 0x04);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x32, 0x03);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x40, 0x83);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x46, 0x25);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x60, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x27, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x50, 0x06);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x51, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x52, 0x96);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x56, 0x08);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x57, 0x30);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x61, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x62, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x64, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x65, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x66, 0xA0);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x22, 0x32);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x47, 0x14);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x49, 0xFF);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x4A, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x7A, 0x0A);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x7B, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x78, 0x21);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x23, 0x34);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x42, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x44, 0xFF);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x45, 0x26);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x46, 0x05);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x40, 0x40);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x0E, 0x06);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x20, 0x1A);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x43, 0x40);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x34, 0x03);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x35, 0x44);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x31, 0x04);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x4B, 0x09);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x4C, 0x05);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x4D, 0x04);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x44, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x45, 0x20);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x47, 0x08);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x48, 0x28);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x67, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x70, 0x04);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x71, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x72, 0xFE);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x76, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x77, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x0D, 0x01);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x01, 0xF8);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x8E, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    i2cdevWriteBit(dev->I2Cx, dev->devAddr, VL53L0X_RA_GPIO_HV_MUX_ACTIVE_HIGH, 4, 0); // active low
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    dev->measurement_timing_budget_us = vl53l0xGetMeasurementTimingBudget(dev);
    dev->measurement_timing_budget_ms = (uint16_t)(dev->measurement_timing_budget_us / 1000.0f);

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    vl53l0xSetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x01);

    if (!vl53l0xPerformSingleRefCalibration(dev, 0x40)) {
        DEBUG_PRINTD("Failed VHV calibration");
        return false;
    }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x02);

    if (!vl53l0xPerformSingleRefCalibration(dev, 0x00)) {
        DEBUG_PRINTD("Failed phase calibration");
        return false;
    }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    return true;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool vl53l0xSetSignalRateLimit(VL53L0xDev *dev, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99f) {
        return false;
    }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    uint16_t fixed_pt = limit_Mcps * (1 << 7);
    return vl53l0xWriteReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, fixed_pt);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool vl53l0xSetMeasurementTimingBudget(VL53L0xDev *dev, uint32_t budget_us)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) {
        return false;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    vl53l0xGetSequenceStepEnables(dev, &enables);
    vl53l0xGetSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us) {
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            vl53l0xTimeoutMicrosecondsToMclks(final_range_timeout_us,
                                              timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        uint16_t temp = vl53l0xEncodeTimeout(final_range_timeout_mclks);
        vl53l0xWriteReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, temp);

        // set_sequence_step_timeout() end

        dev->measurement_timing_budget_us = budget_us; // store for internal reuse
        dev->measurement_timing_budget_ms = (uint16_t)(dev->measurement_timing_budget_us / 1000.0f);
    }

    return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t vl53l0xGetMeasurementTimingBudget(VL53L0xDev *dev)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    vl53l0xGetSequenceStepEnables(dev, &enables);
    vl53l0xGetSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us; // store for internal reuse
    dev->measurement_timing_budget_ms = (uint16_t)(dev->measurement_timing_budget_us / 1000.0f);
    return budget_us;
}


// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool vl53l0xSetVcselPulsePeriod(VL53L0xDev *dev, vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    vl53l0xGetSequenceStepEnables(dev, &enables);
    vl53l0xGetSequenceStepTimeouts(dev, &enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."


    if (type == VcselPeriodPreRange) {
        // "Set phase check limits"
        switch (period_pclks) {
            case 12:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                break;

            case 14:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                break;

            case 16:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                break;

            case 18:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                break;

            default:
                // invalid period
                return false;
        }

        i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks =
            vl53l0xTimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        uint16_t new_pre_range_timeout_encoded = vl53l0xEncodeTimeout(new_pre_range_timeout_mclks);
        vl53l0xWriteReg16Bit(dev, VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_encoded);

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks =
            vl53l0xTimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP,
                        (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    } else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x30);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
                break;

            case 10:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
                break;

            case 12:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
                break;

            case 14:
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
                i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
                break;

            default:
                // invalid period
                return false;
        }

        // apply new VCSEL period
        i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks =
            vl53l0xTimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        uint16_t new_final_range_timeout_encoded = vl53l0xEncodeTimeout(new_final_range_timeout_mclks);
        vl53l0xWriteReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_final_range_timeout_encoded);

        // set_sequence_step_timeout end
    } else {
        // invalid type
        return false;
    }

    // "Finally, the timing budget must be re-applied"

    vl53l0xSetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = 0;
    i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x02);
    bool ret = vl53l0xPerformSingleRefCalibration(dev, 0x0);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return ret;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t vl53l0xGetVcselPulsePeriod(VL53L0xDev *dev, vcselPeriodType type)
{
    if (type == VcselPeriodPreRange) {
        uint8_t temp = 0;
        i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD, &temp);
        return decodeVcselPeriod(temp);
    } else if (type == VcselPeriodFinalRange) {
        uint8_t temp = 0;
        i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &temp);
        return decodeVcselPeriod(temp);
    } else {
        return 255;
    }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void vl53l0xStartContinuous(VL53L0xDev *dev, uint32_t period_ms)
{
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x91, dev->stop_variable);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x00);

    if (period_ms != 0) {
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        uint16_t osc_calibrate_val = vl53l0xReadReg16Bit(dev, VL53L0X_RA_OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0) {
            period_ms *= osc_calibrate_val;
        }

        vl53l0xWriteReg32Bit(dev, VL53L0X_RA_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    } else {
        // continuous back-to-back mode
        i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void vl53l0xStopContinuous(VL53L0xDev *dev)
{
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x91, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t vl53l0xReadRangeContinuousMillimeters(VL53L0xDev *dev)
{
    startTimeout();
    uint8_t val = 0;

    while ((val & 0x07) == 0) {
        i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &val);

        if ((val & 0x07) == 0) {
            // Relaxation delay when polling interrupt
            vTaskDelay(M2T(1));
        }

        if (checkTimeoutExpired()) {
            dev->did_timeout = true;
            return 65535;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    uint16_t range = vl53l0xReadReg16Bit(dev, VL53L0X_RA_RESULT_RANGE_STATUS + 10);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t vl53l0xReadRangeSingleMillimeters(VL53L0xDev *dev)
{
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x91, dev->stop_variable);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    startTimeout();
    uint8_t val = 0x01;

    while (val & 0x01) {
        i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, &val);

        if (checkTimeoutExpired()) {
            dev->did_timeout = true;
            return 65535;
        }
    }

    return vl53l0xReadRangeContinuousMillimeters(dev);
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool vl53l0xGetSpadInfo(VL53L0xDev *dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x06);
    i2cdevWriteBit(dev->I2Cx, dev->devAddr, 0x83, 2, 0x01);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x07);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x81, 0x01);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x01);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x94, 0x6b);
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x83, 0x00);
    startTimeout();

    uint8_t val = 0x00;

    while (val == 0x00) {
        i2cdevReadByte(dev->I2Cx, dev->devAddr, 0x83, &val);

        if (checkTimeoutExpired()) {
            return false;
        }
    };

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x83, 0x01);

    i2cdevReadByte(dev->I2Cx, dev->devAddr, 0x92, &tmp);

    *count = tmp & 0x7f;

    *type_is_aperture = (tmp >> 7) & 0x01;

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x81, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x06);

    i2cdevWriteBit(dev->I2Cx, dev->devAddr, 0x83, 2, 0);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x01);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x00, 0x01);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0xFF, 0x00);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, 0x80, 0x00);

    return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void vl53l0xGetSequenceStepEnables(VL53L0xDev *dev, SequenceStepEnables *enables)
{
    uint8_t sequence_config = 0;
    i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, &sequence_config);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void vl53l0xGetSequenceStepTimeouts(VL53L0xDev *dev, SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(dev, VcselPeriodPreRange);

    uint8_t temp = 0;
    i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP, &temp);
    timeouts->msrc_dss_tcc_mclks = temp + 1;
    timeouts->msrc_dss_tcc_us =
        vl53l0xTimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                          timeouts->pre_range_vcsel_period_pclks);

    uint16_t pre_range_encoded = vl53l0xReadReg16Bit(dev, VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    timeouts->pre_range_mclks = vl53l0xDecodeTimeout(pre_range_encoded);
    timeouts->pre_range_us =
        vl53l0xTimeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                          timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(dev, VcselPeriodFinalRange);

    uint16_t final_range_encoded = vl53l0xReadReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    timeouts->final_range_mclks = vl53l0xDecodeTimeout(final_range_encoded);

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        vl53l0xTimeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                          timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t vl53l0xDecodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
                      (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00LU) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    } else {
        return 0;
    }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
bool vl53l0xPerformSingleRefCalibration(VL53L0xDev *dev, uint8_t vhv_init_byte)
{
    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout();
    uint8_t temp = 0x00;

    while ((temp & 0x07) == 0) {
        i2cdevReadByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &temp);

        if (checkTimeoutExpired()) {
            return false;
        }
    }

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

    i2cdevWriteByte(dev->I2Cx, dev->devAddr, VL53L0X_RA_SYSRANGE_START, 0x00);

    return true;
}

uint16_t vl53l0xReadReg16Bit(VL53L0xDev *dev, uint8_t reg)
{
    uint8_t buffer[2] = {};
    i2cdevReadReg8(dev->I2Cx, dev->devAddr, reg, 2, (uint8_t *)&buffer);
    return ((uint16_t)(buffer[0]) << 8) | buffer[1];
}

bool vl53l0xWriteReg16Bit(VL53L0xDev *dev, uint8_t reg, uint16_t val)
{
    uint8_t buffer[2] = {};
    buffer[0] = ((val >> 8) & 0xFF);
    buffer[1] = ((val) & 0xFF);
    return i2cdevWriteReg8(dev->I2Cx, dev->devAddr, reg, 2, (uint8_t *)&buffer);
}

bool vl53l0xWriteReg32Bit(VL53L0xDev *dev, uint8_t reg, uint32_t val)
{
    uint8_t buffer[4] = {};
    buffer[0] = ((val >> 24) & 0xFF);
    buffer[1] = ((val >> 16) & 0xFF);
    buffer[2] = ((val >>  8) & 0xFF);
    buffer[3] = ((val) & 0xFF);
    return i2cdevWriteReg8(dev->I2Cx, dev->devAddr, reg, 4, (uint8_t *)&buffer);
}
