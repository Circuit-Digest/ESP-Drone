/**
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
 * sound_cf2.c - Module used to play melodies and system sounds though a buzzer
 */

#include <stdbool.h>

/* FreeRtos includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "config.h"
#include "param.h"
#include "log.h"
#include "sound.h"
#include "buzzer.h"
#include "stm32_legacy.h"
#include "sdkconfig.h"

/**
 * Credit to http://tny.cz/e525c1b2 for supplying the tones
 */
#define OFF 0
#define C0 16
#define Db0 17
#define D0  18
#define Eb0 19
#define E0  20
#define F0  21
#define Gb0 23
#define G0  24
#define Ab0 25
#define A0 27
#define Bb0 29
#define B0  30
#define C1  32
#define Db1 34
#define D1  36
#define Eb1 38
#define E1  41
#define F1  43
#define Gb1 46
#define G1  49
#define Ab1 51
#define A1 55
#define Bb1 58
#define B1  61
#define C2  65
#define Db2 69
#define D2  73
#define Eb2 77
#define E2  82
#define F2  87
#define Gb2 92
#define G2  98
#define Ab2 103
#define A2 110
#define Bb2 116
#define B2  123
#define C3  130
#define Db3 138
#define D3  146
#define Eb3 155
#define E3  164
#define F3  174
#define Gb3 185
#define G3  196
#define Ab3 207
#define A3 220
#define Bb3 233
#define B3  246
#define C4  261
#define Db4 277
#define D4  293
#define Eb4 311
#define E4  329
#define F4  349
#define Gb4 369
#define G4  392
#define Ab4 415
#define A4 440
#define Bb4 466
#define B4  493
#define C5  523
#define Db5 554
#define D5  587
#define Eb5 622
#define E5  659
#define F5  698
#define Gb5 739
#define G5  783
#define Ab5 830
#define A5 880
#define Bb5 932
#define B5  987
#define C6  1046
#define Db6 1108
#define D6  1174
#define Eb6 1244
#define E6  1318
#define F6  1396
#define Gb6 1479
#define G6  1567
#define Ab6 1661
#define A6 1760
#define Bb6 1864
#define B6  1975
#define C7  2093
#define Db7 2217
#define D7  2349
#define Eb7 2489
#define E7  2637
#define F7  2793
#define Gb7 2959
#define G7  3135
#define Ab7 3322
#define A7 3520
#define Bb7 3729
#define B7  3951
#define C8  4186
#define Db8 4434
#define D8  4698
#define Eb8 4978
/* Duration of notes */
#define W  1  // 1/1
#define H  2  // 1/2
#define Q  4  // 1/4
#define E  8  // 1/8
#define S  16 // 1/16
#define ES 6
/* End markers */
#define STOP {0xFE, 0}
#define REPEAT {0xFF, 0}

#define MAX_NOTE_LENGTH 80

static bool isInit=false;

typedef const struct {
  uint16_t tone;
  uint16_t duration;
} Note;

typedef const struct {
  uint32_t bpm;
  uint32_t delay;
  Note notes[MAX_NOTE_LENGTH];
} Melody;

static uint32_t neffect = 0;
static uint32_t sys_effect = 0;
static uint32_t user_effect = 0;

static Melody range_slow = {.bpm = 120, .delay = 1, .notes = {{C4, H}, {D4, H}, {E4, H}, {F4, H}, {G4, H}, {A4, H}, {B4, H}, REPEAT}};
static Melody range_fast = {.bpm = 120, .delay = 1, .notes = {{C4, S}, {D4, S}, {E4, S}, {F4, S}, {G4, S}, {A4, S}, {B4, S}, REPEAT}};
static Melody startup = {.bpm = 120, .delay = 1, .notes = {{C6, S}, {C6, S}, STOP}};
static Melody calibrated = {.bpm = 120, .delay = 1, .notes = {{C4, S}, {E4, S}, {G4, S}, {C5, E}, STOP}};
static Melody chg_done = {.bpm = 120, .delay = 1, .notes = {{D4, Q}, {A4, Q}, STOP}};
static Melody lowbatt = {.bpm = 120, .delay = 1, .notes = {{D4, E}, {A4, E}, {D4, E}, REPEAT}};
static Melody usb_disconnect = {.bpm = 120, .delay = 1, .notes = {{C4, E}, STOP}};
static Melody usb_connect = {.bpm = 120, .delay = 1, .notes = {{A4, E}, STOP}};
static Melody factory_test = {.bpm = 120, .delay = 1, .notes = {{A1, Q}, {OFF, S}, {A2, Q}, {OFF, S}, REPEAT}};
static Melody starwars = {.bpm = 120, .delay = 1, .notes = {{A3, Q}, {A3, Q}, {A3, Q}, {F3, ES}, {C4, S},
    {A3, Q}, {F3, ES}, {C4, S}, {A3, H},
    {E4, Q}, {E4, Q}, {E4, Q}, {F4, ES}, {C4, S},
    {Ab3, Q}, {F3, ES}, {C4, S}, {A3, H},
    {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {C4, S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {A3, S},
    {C4, Q}, {A3, ES}, {C4, S}, {E4, H},
    {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {C4,S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {C4, S},
    {A3, Q}, {F3, ES}, {C4, S}, {A3, H}, {0, H},
    REPEAT}};
static Melody valkyries = {.bpm = 140, .delay = 1, .notes = {{Gb3, Q}, {B3, Q},
    {Gb3, S}, {B3, E},  {D4, Q}, {B3, Q}, {D4, Q}, {B3, S}, {D4, E}, {Gb4, Q},
    {D4, Q}, {Gb4, Q}, {D4, S}, {Gb4, E}, {A4, Q}, {A3, Q}, {D4, Q}, {A3, S},
    {D4, E}, {Gb4, H},
    REPEAT}};

typedef void (*BuzzerEffect)(uint32_t timer, uint32_t * mi, Melody * melody);

static void off(uint32_t counter, uint32_t * mi, Melody * m) {
  buzzerOff();
}

static void turnCurrentEffectOff() {
  if (sys_effect != 0) {
    sys_effect = 0;
  } else {
    user_effect = 0;
  }
}

static uint32_t mcounter = 0;
static void melodyplayer(uint32_t counter, uint32_t * mi, Melody * m) {
  uint16_t tone = m->notes[(*mi)].tone;
  uint16_t duration = m->notes[(*mi)].duration;

  if (mcounter == 0) {
    if (tone == 0xFE) {
      // Turn off buzzer since we're at the end
      (*mi) = 0;
      turnCurrentEffectOff();
    } else if (tone == 0xFF) {
      // Loop the melody
      (*mi) = 0;
    } else {
      // Play current note
      buzzerOn(tone);
      mcounter = (100 * 4 * 60) / (m->bpm * duration) - 1;
      (*mi)++;
    }
  } else {
    if (mcounter == 1) {
        buzzerOff();
    }
    mcounter--;
  }
}

static uint8_t static_ratio = 0;
static uint16_t static_freq = 4000;
static void bypass(uint32_t counter, uint32_t * mi, Melody * melody)
{
  buzzerOn(static_freq);
}

static uint16_t siren_start = 2000;
static uint16_t siren_freq = 2000;
static uint16_t siren_stop = 4000;
static int16_t siren_step = 40;
static void siren(uint32_t counter, uint32_t * mi, Melody * melody)
{
  siren_freq += siren_step;
  if (siren_freq > siren_stop) {
    siren_step *= -1;
    siren_freq = siren_stop;
  }
  if (siren_freq < siren_start) {
    siren_step *= -1;
    siren_freq = siren_start;
  }
  buzzerOn(siren_freq);
}

static int pitchid;
static int rollid;
static int pitch;
static int roll;
static int tilt_freq;
static int tilt_ratio;
static void tilt(uint32_t counter, uint32_t * mi, Melody * melody)
{
  pitchid = logGetVarId("stabilizer", "pitch");
  rollid = logGetVarId("stabilizer", "roll");

  pitch = logGetInt(pitchid);
  roll = logGetInt(rollid);
  tilt_freq = 0;
  tilt_ratio = 127;

  if (abs(pitch) > 5) {
    tilt_freq = 3000 - 50 * pitch;
  }

  buzzerOn(tilt_freq);
}

typedef struct {
  BuzzerEffect call;
  uint32_t mi;
  Melody * melody;
} EffectCall;

static EffectCall effects[] = {
    [SND_OFF] = {.call = &off},
    [FACTORY_TEST] = {.call = &melodyplayer, .melody = &factory_test},
    [SND_USB_CONN] = {.call = &melodyplayer, .melody = &usb_connect},
    [SND_USB_DISC] = {.call = &melodyplayer, .melody = &usb_disconnect},
    [SND_BAT_FULL] = {.call = &melodyplayer, .melody = &chg_done},
    [SND_BAT_LOW] = {.call = &melodyplayer, .melody = &lowbatt},
    [SND_STARTUP] = {.call = &melodyplayer, .melody = &startup},
    [SND_CALIB] = {.call = &melodyplayer, .melody = &calibrated},
    {.call = &melodyplayer, .melody = &range_slow},
    {.call = &melodyplayer, .melody = &range_fast},
    {.call = &melodyplayer, .melody = &starwars},
    {.call = &melodyplayer, .melody = &valkyries},
    {.call = &bypass},
    {.call = &siren},
    {.call = &tilt}
};

static xTimerHandle timer;
static StaticTimer_t timerBuffer;
static uint32_t counter = 0;

static void soundTimer(xTimerHandle timer)
{
  int effect;
  counter++;

  if (sys_effect != 0) {
    effect = sys_effect;
  } else {
    effect = user_effect;
  }

  if (effects[effect].call != 0) {
    effects[effect].call(counter * 10, &effects[effect].mi, effects[effect].melody);
  }
}

void soundInit(void)
{
#ifdef CONFIG_BUZZER_ON
  if (isInit) {
    return;
  }

  neffect = sizeof(effects) / sizeof(effects[0]) - 1;

  timer = xTimerCreateStatic("SoundTimer", M2T(10), pdTRUE, NULL, soundTimer, &timerBuffer);
  xTimerStart(timer, 100);

  isInit = true;
#endif
}

bool soundTest(void)
{
  return isInit;
}

void soundSetEffect(uint32_t effect)
{
  sys_effect = effect;
}

void soundSetFreq(uint32_t freq) {

}

PARAM_GROUP_START(sound)
PARAM_ADD(PARAM_UINT8, effect, &user_effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD(PARAM_UINT16, freq, &static_freq)
PARAM_ADD(PARAM_UINT8, ratio, &static_ratio)
PARAM_GROUP_STOP(sound)
