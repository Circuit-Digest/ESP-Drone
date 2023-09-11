/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 */

/*
 * ledseq.h - LED sequence handler
 */

#ifndef __LEDSEQ_H__
#define __LEDSEQ_H__

/* A LED sequence is made of a list of actions. Each action contains the new
 * state of the LED and either a time to wait before executing the next action
 * or a command LOOP or STOP.
 *
 * The sequences are stored in a list by priority order ([0] is the highest
 * priority). The list ordered by priority is defined at the beginning of
 * ledseq.c
 *
 * Each sequence effects only one LED. For each LED only the runnable sequence
 * with the highest priority is run.
 */

#include <stdint.h>
#include <stdbool.h>
#include "led.h"

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500
//Led sequence action
#define LEDSEQ_WAITMS(X) (X)
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2

typedef struct {
  bool value;
  int action;
} ledseqStep_t;

typedef struct ledseqContext_s {
  ledseqStep_t* const sequence;
  struct ledseqContext_s* nextContext;
  int state;
  const led_t led;
} ledseqContext_t;

// Public API
void ledseqInit(void);
bool ledseqTest(void);

/**
 * @brief Enable/disable the led sequence module
 *
 * @param enable Enabled if true
 */
void ledseqEnable(bool enable);

/**
 * @brief Register a new LED sequence. Sequences must be
 *        registerd before they can be used, registration order detemines the
 *        relative priorities between sequences.
 *
 * @param context Pointer to a LED seqence runtime context. Note the
 *        context will be part of linked list of contexts and must exist
 *        as long as the system is running.
 */
void ledseqRegisterSequence(ledseqContext_t* context);

/**
 * @brief Run a LED sequence. This function is non-blocking any may fail to start the sequence.
 *
 * @param context The context for the sequence to start
 * @return true If the sequence was started
 * @return false If the sequence did not start
 */
bool ledseqRun(ledseqContext_t* context);

/**
 * @brief Run a LED sequence. This function is blocking and may take some time,
 *        but is quaranteed to mark the sequence as started.
 *
 * @param context The context for the sequence to start
 */
void ledseqRunBlocking(ledseqContext_t* context);

/**
 * @brief Stop a LED sequence. This function is non-blocking any may fail to stop the sequence.
 *
 * @param context The context for the sequence to stop
 * @return true If the sequence was stopped
 * @return false If the sequence did not stop
 */
bool ledseqStop(ledseqContext_t* context);

/**
 * @brief Stop a LED sequence. This function is blocking and may take some time,
 *        but is quaranteed to mark the sequence as started.
 *
 * @param context The context for the sequence to stop
 */
void ledseqStopBlocking(ledseqContext_t* context);

/**
 * @brief Set interval in the charging sequence to reflect the
 *        charge level
 *
 * @param chargeLevel The charge level between 0.0 and 1.0
 */
void ledseqSetChargeLevel(const float chargeLevel);

// System led sequences
extern ledseqContext_t seq_calibrated;
extern ledseqContext_t seq_alive;
extern ledseqContext_t seq_lowbat;
extern ledseqContext_t seq_linkUp;
extern ledseqContext_t seq_linkDown;
extern ledseqContext_t seq_charged;
extern ledseqContext_t seq_charging;
extern ledseqContext_t seq_testPassed;
extern ledseqContext_t seq_testFailed;

#endif
