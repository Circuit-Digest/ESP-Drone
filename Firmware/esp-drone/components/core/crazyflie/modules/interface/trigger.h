/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * trigger.h - Interface to trigger mechanism.
 */

#ifndef __TRIGGER_H__
#define __TRIGGER_H__

#include <stdint.h>
#include <stdbool.h>

/* Trigger function type. */
typedef enum {
  triggerFuncNone   = 0, /* No trigger function. */
  triggerFuncIsLE   = 1, /* Increases testCounter if test value is Less than or Equal to threshold value. */
  triggerFuncIsGE   = 2, /* Increases testCounter if test value is Greater than or Equal to threshold value. */
} triggerFunc_t;

/* Trigger handler function type. */
typedef void (*triggerHandler_t)(void *);

/**
 * Trigger object.
 *
 * This object is published here so that users may investigate the testCounter and released attributes
 * during debugging. Such attributes can for instance be passed through the LOG framework.
 */
typedef struct {
    bool active;              /* Indicates if the trigger is active or not. */
    triggerFunc_t func;       /* The trigger function type. */
    float threshold;          /* The threshold the test value is compared against using the trigger function. */
    uint32_t triggerCount;    /* When testCounter reaches this value, a trigger is released. */
    triggerHandler_t handler; /* If registered, the handler function is called when the trigger is released. */
    bool handlerCalled;       /* Indicates if the handler has been called. The handler is only called once after the trigger is released. */
    void *handlerArg;         /* The argument to pass to the handler function. */
    uint32_t testCounter;     /* As long as smaller than triggerCount, this counter is incremented for each consecutive test the tested value is within the threshold. */
    bool released;            /* Indicates if the trigger has been released (true) or not (false). */
} trigger_t;

/* Trigger functionality. */
void triggerInit(trigger_t *trigger, triggerFunc_t func, float threshold, uint32_t triggerCount);
void triggerRegisterHandler(trigger_t *trigger, triggerHandler_t handler, void *handlerArg);
void triggerDeInit(trigger_t *trigger);
void triggerActivate(trigger_t *trigger, bool active);
void triggerReset(trigger_t *trigger);
bool triggerTestValue(trigger_t *trigger, float testValue);

#endif
