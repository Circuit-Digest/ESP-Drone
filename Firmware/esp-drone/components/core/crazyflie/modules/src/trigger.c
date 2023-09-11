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
 * trigger.c - Implementation of trigger mechanism.
 */

#include <stddef.h>
#include <stdlib.h>
#include "stm32_legacy.h"
#include "trigger.h"

/**
 * Initialize a trigger object.
 *
 * Note that to activate the trigger, the triggerActivate() function must be called after
 * calling triggerInit().
 *
 * @param trigger      The trigger object.
 * @param func         The trigger function type.
 * @param threshold    The threshold to use with the trigger function.
 * @param triggerCount When testCounter reaches this value, a trigger is reported.
 */
void triggerInit(trigger_t *trigger, triggerFunc_t func, float threshold, uint32_t triggerCount)
{
  assert_param(trigger != NULL);
  assert_param(func != triggerFuncNone);

  trigger->active = false;
  trigger->func = func;
  trigger->threshold = threshold;
  trigger->triggerCount = triggerCount;

  triggerReset(trigger);
}

/**
 * Register a trigger handler.
 *
 * @param trigger      The trigger object.
 * @param handler      If a handler is registered, this handler function is called when the trigger is released. Otherwise NULL.
 * @param handlerArg   Argument to pass to the handler function. NULL if not used.
 */
void triggerRegisterHandler(trigger_t *trigger, triggerHandler_t handler, void *handlerArg)
{
  assert_param(trigger != NULL);
  assert_param(handler != NULL);

  trigger->handler = handler;
  trigger->handlerArg = handlerArg;
  trigger->handlerCalled = false;
}

/**
 * DeInitialize a trigger object.
 *
 * @param trigger The trigger object.
 */
void triggerDeInit(trigger_t *trigger)
{
  assert_param(trigger != NULL);

  triggerInit(trigger, triggerFuncNone, 0, 0);
  triggerRegisterHandler(trigger, NULL, NULL);
  triggerReset(trigger);
}

/**
 * Activate or deactivate a trigger object.
 *
 * This function does not influence the attributes of the trigger object as initialized by
 * triggerInitialize() or triggerRegisterHandler() functions.
 *
 * Calling this function will reset all counters and flags, in addition to activating or
 * deactivating the trigger.
 *
 * @param trigger The trigger object.
 * @param active  Set the trigger active (true) or deactive (false).
 */
void triggerActivate(trigger_t *trigger, bool active)
{
  assert_param(trigger != NULL);

  triggerReset(trigger);
  trigger->active = active;
}

/**
 * Reset a trigger object.
 *
 * This function does not influence the attributes of the trigger object as initialized by
 * triggerInitialize(), triggerRegisterHandler() or triggerActivate() functions.
 *
 * Calling this function will reset all other counters and flags.
 *
 * @param trigger The trigger object.
 */
void triggerReset(trigger_t *trigger)
{
  assert_param(trigger != NULL);

  trigger->testCounter = 0;
  trigger->released = false;
  trigger->handlerCalled = false;
}

/**
 * Internal function incrementing the testCounter, but never further than the triggerCount value.
 *
  * @param trigger The trigger object.
 */
static void triggerIncTestCounter(trigger_t *trigger)
{
  assert_param(trigger != NULL);

   if(trigger->testCounter < trigger->triggerCount) {
     trigger->testCounter++;
   }
}

/**
 * Test a value against the trigger.
 *
 * Note that this function will call the registered handler function synchronously. The handler function
 * is called only once, when the trigger is released. Subsequent calls to this function will not call
 * the handler function again (until triggerReset() or triggerActivate() has been called to reset the
 * trigger).
 *
 * This function will continue to return a value of true as long as the condition for the trigger release
 * has been met (or exceeds the triggerCount value).
 *
 * This function will increment the testCounter until it reaches triggerCount.
 *
 * @param trigger   The trigger object.
 * @param testValue The test value to compare against the threshold with the trigger function.
 *
 * @return True when the trigger has been released.
 */
bool triggerTestValue(trigger_t *trigger, float testValue)
{
  assert_param(trigger != NULL);

  /* Do not do anything if the trigger has been deactivated. */
  if(!trigger->active) {
    return false;
  }

  switch(trigger->func) {
    case triggerFuncIsLE: {
      if(testValue <= trigger->threshold) {
        triggerIncTestCounter(trigger);
        break;
      }
      else {
        /* Reset the trigger if the test failed. */
        triggerReset(trigger);
        return false;
      }
    }
    case triggerFuncIsGE: {
      if(testValue >= trigger->threshold) {
        triggerIncTestCounter(trigger);
        break;
      }
      else {
        /* Reset the trigger if the test failed. */
        triggerReset(trigger);
        return false;
      }
    }
    case triggerFuncNone: {
      /* No action, included to avoid compiler warnings. */
      break;
    }
  }

  /* Check if the triggerCount has been reached. */
  trigger->released = (trigger->testCounter >= trigger->triggerCount);

  /* If the trigger has not been release, exit immediately. */
  if(!trigger->released) {
    return false;
  }

  /* The trigger object may be reset by the handler, thus make an internal copy of the released flag. */
  bool iReleased = trigger->released;
  if(trigger->released && (trigger->handler != NULL) && (!trigger->handlerCalled)) {
    /* Set the handlerCalled = true before calling, since the handler may choose to reset the object. */
    trigger->handlerCalled = true;
    trigger->handler(trigger->handlerArg);
  }

  /* Return the release flag. This may return the value true more than once. */
  return iReleased;
}
