/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouse_calibration.c: lighthouse positioning angle calibration
 */

#include "lighthouse_calibration.h"

#include <math.h>

// Enable to use only phase for calibration. The more complex calibration does not
// seem to give as good results as the simple one(!)
#define USE_SIMPLE_CALIBRATION

void lighthouseCalibrationInitFromFrame(lighthouseCalibration_t *calib, struct ootxDataFrame_s *frame)
{
  calib->axis[0].phase = frame->phase0;
  calib->axis[0].tilt = frame->tilt0;
  calib->axis[0].curve = frame->curve0;
  calib->axis[0].gibmag = frame->gibmag0;
  calib->axis[0].gibphase = frame->gibphase0;

  calib->axis[1].phase = frame->phase1;
  calib->axis[1].tilt = frame->tilt1;
  calib->axis[1].curve = frame->curve1;
  calib->axis[1].gibmag = frame->gibmag1;
  calib->axis[1].gibphase = frame->gibphase1;

  calib->valid = true;
}

#ifndef USE_SIMPLE_CALIBRATION
// Calibration function inspired from https://github.com/cnlohr/libsurvive/issues/18#issuecomment-386190279

// Given a predicted sensor position in the lighthouse frame, predict the perturbed measurements
static void predict(const lighthouseCalibration_t* calib, float const* xy, float* ang) {
  float tiltX = calib->axis[0].tilt;
  float phaseX = calib->axis[0].phase;
  float curveX = calib->axis[0].curve;
  float gibmagX = calib->axis[0].gibmag;
  float gibphaseX = calib->axis[0].gibphase;

  float tiltY = calib->axis[1].tilt;
  float phaseY = calib->axis[1].phase;
  float curveY = calib->axis[1].curve;
  float gibmagY = calib->axis[1].gibmag;
  float gibphaseY = calib->axis[1].gibphase;

  ang[0] = atanf(xy[0] - (tiltX + curveX * xy[1]) * xy[1]);
  ang[1] = atanf(xy[1] - (tiltY + curveY * xy[0]) * xy[0]);
  ang[0] -= phaseX + gibmagX * sinf(ang[0] + gibphaseX);
  ang[1] -= phaseY + gibmagY * sinf(ang[1] + gibphaseY);
}

// Given the perturbed lighthouse angle, predict the ideal angle
static void correct(const lighthouseCalibration_t* calib, const float * angle, float * corrected) {
  float ideal[2], pred[2], xy[2];
  ideal[0] = angle[0];
  ideal[1] = angle[1];
  for (int i = 0; i < 10; i++) {
    xy[0] = tanf(ideal[0]);
    xy[1] = tanf(ideal[1]);
    predict(calib, xy, pred);
    ideal[0] += (angle[0] - pred[0]);
    ideal[1] += (angle[1] - pred[1]);
  }
  corrected[0] = ideal[0];
  corrected[1] = ideal[1];
}
#endif

void lighthouseCalibrationApply(lighthouseCalibration_t* calib, float rawAngles[2], float correctedAngles[2])
{
  if (calib->valid) {
    #ifdef USE_SIMPLE_CALIBRATION
    correctedAngles[0] = rawAngles[0] + calib->axis[0].phase;
    correctedAngles[1] = rawAngles[1] + calib->axis[1].phase;
    #else
    correct(calib, rawAngles, correctedAngles);
    #endif
  } else {
    correctedAngles[0] = rawAngles[0];
    correctedAngles[1] = rawAngles[1];
  }
}
