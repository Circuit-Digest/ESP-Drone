/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_common_tables.h
 * Description:  Extern declaration for common tables
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 XTENSA Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _XTENSA_COMMON_TABLES_H
#define _XTENSA_COMMON_TABLES_H

#include "xtensa_math.h"

extern const uint16_t xtensaBitRevTable[1024];
extern const float32_t twiddleCoef_16[32];
extern const float32_t twiddleCoef_32[64];
extern const float32_t twiddleCoef_64[128];
extern const float32_t twiddleCoef_128[256];
extern const float32_t twiddleCoef_256[512];
extern const float32_t twiddleCoef_512[1024];
extern const float32_t twiddleCoef_1024[2048];
extern const float32_t twiddleCoef_2048[4096];
extern const float32_t twiddleCoef_4096[8192];
#define twiddleCoef twiddleCoef_4096
extern const float32_t twiddleCoef_rfft_32[32];
extern const float32_t twiddleCoef_rfft_64[64];
extern const float32_t twiddleCoef_rfft_128[128];
extern const float32_t twiddleCoef_rfft_256[256];
extern const float32_t twiddleCoef_rfft_512[512];
extern const float32_t twiddleCoef_rfft_1024[1024];
extern const float32_t twiddleCoef_rfft_2048[2048];
extern const float32_t twiddleCoef_rfft_4096[4096];

/* floating-point bit reversal tables */
#define XTENSABITREVINDEXTABLE_16_TABLE_LENGTH ((uint16_t)20)
#define XTENSABITREVINDEXTABLE_32_TABLE_LENGTH ((uint16_t)48)
#define XTENSABITREVINDEXTABLE_64_TABLE_LENGTH ((uint16_t)56)
#define XTENSABITREVINDEXTABLE_128_TABLE_LENGTH ((uint16_t)208)
#define XTENSABITREVINDEXTABLE_256_TABLE_LENGTH ((uint16_t)440)
#define XTENSABITREVINDEXTABLE_512_TABLE_LENGTH ((uint16_t)448)
#define XTENSABITREVINDEXTABLE_1024_TABLE_LENGTH ((uint16_t)1800)
#define XTENSABITREVINDEXTABLE_2048_TABLE_LENGTH ((uint16_t)3808)
#define XTENSABITREVINDEXTABLE_4096_TABLE_LENGTH ((uint16_t)4032)

extern const uint16_t xtensaBitRevIndexTable16[XTENSABITREVINDEXTABLE_16_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable32[XTENSABITREVINDEXTABLE_32_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable64[XTENSABITREVINDEXTABLE_64_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable128[XTENSABITREVINDEXTABLE_128_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable256[XTENSABITREVINDEXTABLE_256_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable512[XTENSABITREVINDEXTABLE_512_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable1024[XTENSABITREVINDEXTABLE_1024_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable2048[XTENSABITREVINDEXTABLE_2048_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable4096[XTENSABITREVINDEXTABLE_4096_TABLE_LENGTH];

/* fixed-point bit reversal tables */
#define XTENSABITREVINDEXTABLE_FIXED_16_TABLE_LENGTH ((uint16_t)12)
#define XTENSABITREVINDEXTABLE_FIXED_32_TABLE_LENGTH ((uint16_t)24)
#define XTENSABITREVINDEXTABLE_FIXED_64_TABLE_LENGTH ((uint16_t)56)
#define XTENSABITREVINDEXTABLE_FIXED_128_TABLE_LENGTH ((uint16_t)112)
#define XTENSABITREVINDEXTABLE_FIXED_256_TABLE_LENGTH ((uint16_t)240)
#define XTENSABITREVINDEXTABLE_FIXED_512_TABLE_LENGTH ((uint16_t)480)
#define XTENSABITREVINDEXTABLE_FIXED_1024_TABLE_LENGTH ((uint16_t)992)
#define XTENSABITREVINDEXTABLE_FIXED_2048_TABLE_LENGTH ((uint16_t)1984)
#define XTENSABITREVINDEXTABLE_FIXED_4096_TABLE_LENGTH ((uint16_t)4032)

extern const uint16_t xtensaBitRevIndexTable_fixed_16[XTENSABITREVINDEXTABLE_FIXED_16_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_32[XTENSABITREVINDEXTABLE_FIXED_32_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_64[XTENSABITREVINDEXTABLE_FIXED_64_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_128[XTENSABITREVINDEXTABLE_FIXED_128_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_256[XTENSABITREVINDEXTABLE_FIXED_256_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_512[XTENSABITREVINDEXTABLE_FIXED_512_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_1024[XTENSABITREVINDEXTABLE_FIXED_1024_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_2048[XTENSABITREVINDEXTABLE_FIXED_2048_TABLE_LENGTH];
extern const uint16_t xtensaBitRevIndexTable_fixed_4096[XTENSABITREVINDEXTABLE_FIXED_4096_TABLE_LENGTH];

/* Tables for Fast Math Sine and Cosine */
extern const float32_t sinTable_f32[FAST_MATH_TABLE_SIZE + 1];

#endif /*  XTENSA_COMMON_TABLES_H */
