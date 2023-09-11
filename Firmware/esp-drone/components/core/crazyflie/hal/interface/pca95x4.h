/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2017 Bitcraze AB
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
 * pca95x4.h - Functions for interfacing PCA95X4 I2C GPIO extender
 */
#ifndef __PCA95X4_H__
#define __PCA95X4_H__

#include <stdint.h>
#include <stdbool.h>

#define PCA95X4_DEFAULT_ADDRESS 0b0100000

#define PCA95X4_INPUT_REG   (0x00)
#define PCA95X4_OUTPUT_REG  (0x01)
#define PCA95X4_POL_REG     (0x02)
#define PCA95X4_CONFIG_REG  (0x03)

#define PCA95X4_P0 (1 << 0)
#define PCA95X4_P1 (1 << 1)
#define PCA95X4_P2 (1 << 2)
#define PCA95X4_P3 (1 << 3)
#define PCA95X4_P4 (1 << 4)
#define PCA95X4_P5 (1 << 5)
#define PCA95X4_P6 (1 << 6)
#define PCA95X4_P7 (1 << 7)

/**
 * Initilize the PCA95X4 sub-system.
 */
void pca95x4Init();

/**
 * Test the PCA95X4 sub-system.
 */
bool pca95x4Test();

/**
 * Set the output register value.
 */
bool pca95x4ConfigOutput(uint32_t val);

/**
 * Set output bits.
 */
bool pca95x4SetOutput(uint32_t mask);

/**
 * Reset output bits.
 */
bool pca95x4ClearOutput(uint32_t mask);

#endif //__PCA95X4_H__
