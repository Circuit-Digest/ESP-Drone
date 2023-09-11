/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 *
 * pulse_processor_v2.c - pulse decoding for lighthouse V2 base stations
 *
 */

#include "pulse_processor_v2.h"

#include <string.h>
#include <math.h>
#include "math3d.h"
#include "test_support.h"

static const uint32_t MAX_TICKS_SENSOR_TO_SENSOR = 10000;
static const uint32_t MAX_TICKS_BETWEEN_SWEEPS = 220000;
static const uint8_t NO_CHANNEL = 0xff;
static const int NO_SENSOR = -1;
static const uint32_t NO_OFFSET = 0;

#define V2_N_CHANNELS 16

// The cycle times come from the base stations and are expressed in a 48 MHz clock, we use 24 MHz clock hence the "/ 2".
static const uint32_t CYCLE_PERIODS[V2_N_CHANNELS] = {
    959000 / 2, 957000 / 2, 953000 / 2, 949000 / 2,
    947000 / 2, 943000 / 2, 941000 / 2, 939000 / 2,
    937000 / 2, 929000 / 2, 919000 / 2, 911000 / 2,
    907000 / 2, 901000 / 2, 893000 / 2, 887000 / 2
};

TESTABLE_STATIC bool processBlock(const pulseProcessorV2PulseWorkspace_t* pulseWorkspace, pulseProcessorV2SweepBlock_t* block) {
    // Check we have data for all sensors
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        if (! pulseWorkspace->sensors[i].isSet) {
            // The sensor data is missing - discard
            return false;
        }
    }

    // Channel - should all be the same except one that is not set
    int channel_count = 0;
    block->channel = NO_CHANNEL;
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorV2Pulse_t* sensor = &pulseWorkspace->sensors[i];

        if (sensor->channelFound) {
            channel_count++;

            if (block->channel == NO_CHANNEL) {
                block->channel = sensor->channel;
                block->slowbit = sensor->slowbit;
            }

            if (block->channel != sensor->channel) {
                // Multiple channels in the block - discard
                return false;
            }

            if (block->slowbit != sensor->slowbit) {
                // Multiple slowbits in the block - discard
                return false;
            }
        }
    }

    if (channel_count != (PULSE_PROCESSOR_N_SENSORS - 1)) {
        // Channel is missing - discard
        return false;
    }

    // Offset - should be offset on one and only one sensor
    int indexWithOffset = NO_SENSOR;
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorV2Pulse_t* sensor = &pulseWorkspace->sensors[i];
        if (sensor->offset != NO_OFFSET) {
            if (indexWithOffset == NO_SENSOR) {
                indexWithOffset = i;
            } else {
                // Duplicate offsets - discard
                return false;
            }
        }
    }

    if (indexWithOffset == NO_SENSOR) {
        // No offset found - discard
        return false;
    }

    // Calculate offsets for all sensors
    const pulseProcessorV2Pulse_t* baseSensor = &pulseWorkspace->sensors[indexWithOffset];
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorV2Pulse_t* sensor = &pulseWorkspace->sensors[i];
        if (i == indexWithOffset) {
            block->offset[i] = sensor->offset;
        } else {
            uint32_t timestamp_delta = TS_DIFF(baseSensor->timestamp, sensor->timestamp);
            block->offset[i] = TS_DIFF(baseSensor->offset, timestamp_delta);
        }
    }

    block->timestamp = pulseWorkspace->sensors[0].timestamp;

    return true;
}

static bool storePulse(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace) {
    const uint8_t sensor = frameData->sensor;
    pulseProcessorV2Pulse_t* storage = &pulseWorkspace->sensors[sensor];

    bool result = false;
    if (! storage->isSet) {
        storage->timestamp = frameData->timestamp;
        storage->offset = frameData->offset;
        storage->channel = frameData->channel;
        storage->slowbit = frameData->slowbit;
        storage->channelFound = frameData->channelFound;

        storage->isSet = true;
        result = true;
    }

    return result;
}

TESTABLE_STATIC void clearWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace) {
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        pulseWorkspace->sensors[i].isSet = false;
    }
}

static bool processFrame(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace, pulseProcessorV2SweepBlock_t* block) {
    bool result = false;

    uint32_t delta = TS_DIFF(frameData->timestamp, pulseWorkspace->latestTimestamp);
    bool isEndOfPreviuosBlock = (delta > MAX_TICKS_SENSOR_TO_SENSOR);
    if (isEndOfPreviuosBlock) {
        result = processBlock(pulseWorkspace, block);
        clearWorkspace(pulseWorkspace);
    }

    pulseWorkspace->latestTimestamp = frameData->timestamp;

    if (! storePulse(frameData, pulseWorkspace)) {
        clearWorkspace(pulseWorkspace);
    }

    return result;
}

static void calculateAzimuthElevation(const float firstBeam, const float secondBeam, float* angles) {
    const float a120 = M_PI_F * 120.0f / 180.0f;
    const float tan_p_2 = 0.5773502691896258f;   // tan(60 / 2)

    angles[0] = ((firstBeam + secondBeam) / 2.0f) - M_PI_F;
    float beta = (secondBeam - firstBeam) - a120;
    angles[1] = atan(sinf(beta / 2.0f) / tan_p_2);
}

static void calculateAngles(const pulseProcessorV2SweepBlock_t* latestBlock, const pulseProcessorV2SweepBlock_t* previousBlock, pulseProcessorResult_t* angles) {
    const uint8_t channel = latestBlock->channel;

    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        uint32_t firstOffset = previousBlock->offset[i];
        uint32_t secondOffset = latestBlock->offset[i];
        uint32_t period = CYCLE_PERIODS[channel];

        float firstBeam = firstOffset * 2 * M_PI_F / period;
        float secondBeam = secondOffset * 2 * M_PI_F / period;

        calculateAzimuthElevation(firstBeam, secondBeam, angles->sensorMeasurements[i].baseStatonMeasurements[channel].angles);
        angles->sensorMeasurements[i].baseStatonMeasurements[channel].validCount = 2;
    }
}

TESTABLE_STATIC bool isBlockPairGood(const pulseProcessorV2SweepBlock_t* latest, pulseProcessorV2SweepBlock_t* storage) {
    if (latest->channel != storage->channel) {
        return false;
    }

    if (latest->offset[0] < storage->offset[0]) {
        return false;
    }

    if (TS_DIFF(latest->timestamp, storage->timestamp) > MAX_TICKS_BETWEEN_SWEEPS) {
        return false;
    }

    return true;
}

bool pulseProcessorV2ProcessPulse(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis) {
    bool anglesMeasured = false;
    pulseProcessorV2SweepBlock_t block;
    if (processFrame(frameData, &state->pulseWorkspace, &block)) {
        const uint8_t channel = block.channel;
        if (channel < PULSE_PROCESSOR_N_BASE_STATIONS) {
            pulseProcessorV2SweepBlock_t* previousBlock = &state->blocksV2[channel];
            if (isBlockPairGood(&block, previousBlock)) {
                calculateAngles(&block, previousBlock, angles);

                *baseStation = block.channel;
                *axis = sweepDirection_y;
                anglesMeasured = true;
            } else {
                memcpy(previousBlock, &block, sizeof(block));
            }
        }
    }

    return anglesMeasured;
}
