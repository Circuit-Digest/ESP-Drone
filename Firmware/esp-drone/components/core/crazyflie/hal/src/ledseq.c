/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2020 Bitcraze AB
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
 * ledseq.c - LED sequence handler
 */

#include <stdbool.h>

#include "ledseq.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "static_mem.h"

#include "led.h"
#include "stm32_legacy.h"
#include "config.h"

#ifdef CALIBRATED_LED_MORSE
  #define DOT 100
  #define DASH (3 * DOT)
  #define GAP DOT
  #define LETTER_GAP (3 * DOT)
  #define WORD_GAP (7 * DOT)
#endif // #ifdef CALIBRATED_LED_MORSE

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500

/* Led sequences */
ledseqStep_t seq_lowbat_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_lowbat = {
  .sequence = seq_lowbat_def,
  .led = LOWBAT_LED,
};

#define NO_CONTEXT 0
ledseqContext_t* sequences = NO_CONTEXT;

ledseqStep_t seq_calibrated_def[] = {
#ifndef CALIBRATED_LED_MORSE
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(450)},
  {    0, LEDSEQ_LOOP},
#else
  { true, LEDSEQ_WAITMS(DASH)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DASH)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(LETTER_GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DASH)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(WORD_GAP)},
  {    0, LEDSEQ_LOOP},
#endif // ifndef CALIBRATED_LED_MORSE
};

ledseqContext_t seq_calibrated = {
  .sequence = seq_calibrated_def,
  .led = SYS_LED,
};

ledseqStep_t seq_alive_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(1950)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_alive = {
  .sequence = seq_alive_def,
  .led = SYS_LED,
};

ledseqStep_t seq_linkup_def[] = {
  { true, LEDSEQ_WAITMS(1)},
  {false, LEDSEQ_WAITMS(0)},
  {    0, LEDSEQ_STOP},
};

ledseqContext_t seq_linkUp = {
  .sequence = seq_linkup_def,
  .led = LINK_LED,
};

ledseqContext_t seq_linkDown = {
  .sequence = seq_linkup_def,
  .led = LINK_DOWN_LED,
};

ledseqStep_t seq_charged_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_charged = {
  .sequence = seq_charged_def,
  .led = CHG_LED,
};

ledseqStep_t seq_charging_def[] = {
  { true, LEDSEQ_WAITMS(200)},
  {false, LEDSEQ_WAITMS(800)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_charging = {
  .sequence = seq_charging_def,
  .led = CHG_LED,
};

ledseqStep_t seq_testPassed_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_STOP},
};

ledseqContext_t seq_testPassed = {
  .sequence = seq_testPassed_def,
  .led = LINK_LED,
};

ledseqContext_t seq_testFailed = {
  .sequence = seq_testPassed_def,
  .led = SYS_LED,
};

struct ledseqCmd_s {
  enum {run, stop} command;
  ledseqContext_t *sequence;
};

/* Led sequence handling machine implementation */
static void runLedseq(xTimerHandle xTimer);
static void updateActive(led_t led);

NO_DMA_CCM_SAFE_ZERO_INIT static ledseqContext_t* activeSeq[LED_NUM];

NO_DMA_CCM_SAFE_ZERO_INIT static xTimerHandle timer[LED_NUM];
NO_DMA_CCM_SAFE_ZERO_INIT static StaticTimer_t timerBuffer[LED_NUM];

static xSemaphoreHandle ledseqMutex;
static xQueueHandle ledseqCmdQueue;

static bool isInit = false;
static bool ledseqEnabled = false;

static void lesdeqCmdTask(void* param);

void ledseqInit() {
  if(isInit) {
    return;
  }

  ledInit();

  /* Led sequence priority */
  ledseqRegisterSequence(&seq_testPassed);
  ledseqRegisterSequence(&seq_testFailed);
  ledseqRegisterSequence(&seq_lowbat);
  ledseqRegisterSequence(&seq_charged);
  ledseqRegisterSequence(&seq_charging);
  ledseqRegisterSequence(&seq_calibrated);
  ledseqRegisterSequence(&seq_alive);
  ledseqRegisterSequence(&seq_linkUp);
  ledseqRegisterSequence(&seq_linkDown);

  //Initialise the sequences state
  for(int i=0; i<LED_NUM; i++) {
    activeSeq[i] = 0;
  }

  //Init the soft timers that runs the led sequences for each leds
  for(int i=0; i<LED_NUM; i++) {
    timer[i] = xTimerCreateStatic("ledseqTimer", M2T(1000), pdFALSE, (void*)i, runLedseq, &timerBuffer[i]);
  }

  ledseqMutex = xSemaphoreCreateMutex();

  ledseqCmdQueue = xQueueCreate(10, sizeof(struct ledseqCmd_s));
  xTaskCreate(lesdeqCmdTask, LEDSEQCMD_TASK_NAME, LEDSEQCMD_TASK_STACKSIZE, NULL, LEDSEQCMD_TASK_PRI, NULL);

  isInit = true;
}

static void lesdeqCmdTask(void* param) {
  struct ledseqCmd_s command;
  while(1) {
    xQueueReceive(ledseqCmdQueue, &command, portMAX_DELAY);

    switch(command.command) {
      case run:
        ledseqRunBlocking(command.sequence);
        break;
      case stop:
        ledseqStopBlocking(command.sequence);
        break;
    }
  }
}

bool ledseqTest(void) {
  bool status;

  status = isInit & ledTest();
  #ifdef TURN_OFF_LEDS
  ledseqEnable(false);
  ledSet(LED_BLUE_L, 0);
  #else
  ledseqEnable(true);
  #endif

  return status;
}

void ledseqEnable(bool enable) {
  ledseqEnabled = enable;
}

bool ledseqRun(ledseqContext_t *context) {
  struct ledseqCmd_s command;
  command.command = run;
  command.sequence = context;
  if (xQueueSend(ledseqCmdQueue, &command, 0) == pdPASS) {
    return true;
  }
  return false;
}

void ledseqRunBlocking(ledseqContext_t *context) {
  const led_t led = context->led;

  xSemaphoreTake(ledseqMutex, portMAX_DELAY);
  context->state = 0;  //Reset the seq. to its first step
  updateActive(led);
  xSemaphoreGive(ledseqMutex);

  // Run the first step if the new seq is the active sequence
  if(activeSeq[led] == context) {
    runLedseq(timer[led]);
  }
}

void ledseqSetChargeLevel(const float chargeLevel) {
  int onTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA * chargeLevel;
  int offTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA - onTime;

  seq_charging.sequence[0].action = onTime;
  seq_charging.sequence[1].action = offTime;
}

bool ledseqStop(ledseqContext_t *context) {
  struct ledseqCmd_s command;
  command.command = stop;
  command.sequence = context;
  if (xQueueSend(ledseqCmdQueue, &command, 0) == pdPASS) {
    return true;
  }
  return false;
}

void ledseqStopBlocking(ledseqContext_t *context) {
  const led_t led = context->led;

  xSemaphoreTake(ledseqMutex, portMAX_DELAY);
  context->state = LEDSEQ_STOP;  //Stop the seq.
  updateActive(led);
  xSemaphoreGive(ledseqMutex);

  //Run the next active sequence (if any...)
  runLedseq(timer[led]);
}

/* Center of the led sequence machine. This function is executed by the FreeRTOS
 * timers and runs the sequences
 */
static void runLedseq( xTimerHandle xTimer ) {
  if (!ledseqEnabled) {
    return;
  }

  led_t led = (led_t)pvTimerGetTimerID(xTimer);
  ledseqContext_t* context = activeSeq[led];
  if (NO_CONTEXT == context) {
    return;
  }

  bool leave = false;
  while(!leave) {
    if (context->state == LEDSEQ_STOP) {
      return;
    }

    const ledseqStep_t* step = &context->sequence[context->state];

    xSemaphoreTake(ledseqMutex, portMAX_DELAY);
    context->state++;
    led_t led = context->led;

    switch(step->action) {
      case LEDSEQ_LOOP:
        context->state = 0;
        break;
      case LEDSEQ_STOP:
        context->state = LEDSEQ_STOP;
        updateActive(led);
        break;
      default:  //The step is a LED action and a time
        ledSet(led, step->value);
        if (step->action == 0) {
          break;
        }
        xTimerChangePeriod(xTimer, M2T(step->action), 0);
        xTimerStart(xTimer, 0);
        leave = true;
        break;
    }
    xSemaphoreGive(ledseqMutex);
  }
}

void ledseqRegisterSequence(ledseqContext_t* context) {
  context->state = LEDSEQ_STOP;
  context->nextContext = NO_CONTEXT;

  if (sequences == NO_CONTEXT) {
    sequences = context;
  } else {
    ledseqContext_t* last = sequences;
    if (last == context) {
      // Skip if already registered
      return;
    }

    while (last->nextContext != NO_CONTEXT) {
      last = last->nextContext;
      if (last == context) {
        // Skip if already registered
        return;
      }
    }

    last->nextContext = context;
  }
}

// Utility functions

static void updateActive(led_t led) {
  activeSeq[led] = NO_CONTEXT;
  ledSet(led, false);

  for (ledseqContext_t* sequence = sequences; sequence != 0; sequence = sequence->nextContext) {
    if (sequence->led == led && sequence->state != LEDSEQ_STOP) {
      activeSeq[led] = sequence;
      break;
    }
  }
}
