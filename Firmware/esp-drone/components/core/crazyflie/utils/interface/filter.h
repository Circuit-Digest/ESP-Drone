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
 *
 * filter.h - Filtering functions
 */
#ifndef FILTER_H_
#define FILTER_H_
#include <stdint.h>
#include "math.h"

#define IIR_SHIFT         8

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} lpf2pData;

void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);

/** Second order low pass filter structure.
 *
 * using biquad filter with bilinear z transform
 *
 * http://en.wikipedia.org/wiki/Digital_biquad_filter
 * http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform
 *
 * Laplace continious form:
 *
 *                 1
 * H(s) = -------------------
 *        s^2/w^2 + s/w*Q + 1
 *
 *
 * Polynomial discrete form:
 *
 *        b0 + b1 z^-1 + b2 z^-2
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2
 *
 * with:
 *  a0 = 1
 *  a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
 *  a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
 *  b0 = K^2 / (K^2 + K/Q + 1)
 *  b1 = 2*b0
 *  b2 = b0
 *  K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau)
 *  Fc: cutting frequency
 *  Fs: sampling frequency
 *  Ts: sampling period
 *  tau: time constant (tau = 1/(2*pi*Fc))
 *  Q: gain at cutoff frequency
 *
 * Note that b[0]=b[2], so we don't need to save b[2]
 */
struct SecondOrderLowPass {
  float a[2]; ///< denominator gains
  float b[2]; ///< numerator gains
  float i[2]; ///< input history
  float o[2]; ///< output history
};

/** Init second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param Q Q value of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_second_order_low_pass(struct SecondOrderLowPass *filter, float tau, float Q, float sample_time,
    float value)
{
  float K = tanf(sample_time / (2.0f * tau));
  float poly = K * K + K / Q + 1.0f;
  filter->a[0] = 2.0f * (K * K - 1.0f) / poly;
  filter->a[1] = (K * K - K / Q + 1.0f) / poly;
  filter->b[0] = K * K / poly;
  filter->b[1] = 2.0f * filter->b[0];
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/** Update second order low pass filter state with a new value.
 *
 * @param filter second order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_second_order_low_pass(struct SecondOrderLowPass *filter, float value)
{
  float out = filter->b[0] * value
              + filter->b[1] * filter->i[0]
              + filter->b[0] * filter->i[1]
              - filter->a[0] * filter->o[0]
              - filter->a[1] * filter->o[1];
  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[1] = filter->o[0];
  filter->o[0] = out;
  return out;
}

/** Get current value of the second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @return current value of the filter
 */
static inline float get_second_order_low_pass(struct SecondOrderLowPass *filter)
{
  return filter->o[0];
}

struct SecondOrderLowPass_int {
  int32_t a[2]; ///< denominator gains
  int32_t b[2]; ///< numerator gains
  int32_t i[2]; ///< input history
  int32_t o[2]; ///< output history
  int32_t loop_gain; ///< loop gain
};

/** Second order Butterworth low pass filter.
 */
typedef struct SecondOrderLowPass Butterworth2LowPass;

/** Init a second order Butterworth filter.
 *
 * based on the generic second order filter
 * with Q = 0.7071 = 1/sqrt(2)
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter second order Butterworth low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_butterworth_2_low_pass(Butterworth2LowPass *filter, float tau, float sample_time, float value)
{
  init_second_order_low_pass((struct SecondOrderLowPass *)filter, tau, 0.7071, sample_time, value);
}

/** Update second order Butterworth low pass filter state with a new value.
 *
 * @param filter second order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_butterworth_2_low_pass(Butterworth2LowPass *filter, float value)
{
  return update_second_order_low_pass((struct SecondOrderLowPass *)filter, value);
}

/** Get current value of the second order Butterworth low pass filter.
 *
 * @param filter second order Butterworth low pass filter structure
 * @return current value of the filter
 */
static inline float get_butterworth_2_low_pass(Butterworth2LowPass *filter)
{
  return filter->o[0];
}

#endif //FILTER_H_
