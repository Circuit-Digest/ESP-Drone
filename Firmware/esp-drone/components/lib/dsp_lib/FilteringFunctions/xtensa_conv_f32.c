/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_conv_f32.c
 * Description:  Convolution of floating-point sequences
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
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

#include "xtensa_math.h"

/**
 * @ingroup groupFilters
 */

/**
 * @defgroup Conv Convolution
 *
 * Convolution is a mathematical operation that operates on two finite length vectors to generate a finite length output vector.
 * Convolution is similar to correlation and is frequently used in filtering and data analysis.
 * The CMSIS DSP library contains functions for convolving Q7, Q15, Q31, and floating-point data types.
 * The library also provides fast versions of the Q15 and Q31 functions on Cortex-M4 and Cortex-M3.
 *
 * \par Algorithm
 * Let <code>a[n]</code> and <code>b[n]</code> be sequences of length <code>srcALen</code> and <code>srcBLen</code> samples respectively.
 * Then the convolution
 *
 * <pre>
 *                   c[n] = a[n] * b[n]
 * </pre>
 *
 * \par
 * is defined as
 * \image html ConvolutionEquation.gif
 * \par
 * Note that <code>c[n]</code> is of length <code>srcALen + srcBLen - 1</code> and is defined over the interval <code>n=0, 1, 2, ..., srcALen + srcBLen - 2</code>.
 * <code>pSrcA</code> points to the first input vector of length <code>srcALen</code> and
 * <code>pSrcB</code> points to the second input vector of length <code>srcBLen</code>.
 * The output result is written to <code>pDst</code> and the calling function must allocate <code>srcALen+srcBLen-1</code> words for the result.
 *
 * \par
 * Conceptually, when two signals <code>a[n]</code> and <code>b[n]</code> are convolved,
 * the signal <code>b[n]</code> slides over <code>a[n]</code>.
 * For each offset \c n, the overlapping portions of a[n] and b[n] are multiplied and summed together.
 *
 * \par
 * Note that convolution is a commutative operation:
 *
 * <pre>
 *                   a[n] * b[n] = b[n] * a[n].
 * </pre>
 *
 * \par
 * This means that switching the A and B arguments to the convolution functions has no effect.
 *
 * <b>Fixed-Point Behavior</b>
 *
 * \par
 * Convolution requires summing up a large number of intermediate products.
 * As such, the Q7, Q15, and Q31 functions run a risk of overflow and saturation.
 * Refer to the function specific documentation below for further details of the particular algorithm used.
 *
 *
 * <b>Fast Versions</b>
 *
 * \par
 * Fast versions are supported for Q31 and Q15.  Cycles for Fast versions are less compared to Q31 and Q15 of conv and the design requires
 * the input signals should be scaled down to avoid intermediate overflows.
 *
 *
 * <b>Opt Versions</b>
 *
 * \par
 * Opt versions are supported for Q15 and Q7.  Design uses internal scratch buffer for getting good optimisation.
 * These versions are optimised in cycles and consumes more memory(Scratch memory) compared to Q15 and Q7 versions
 */

/**
 * @addtogroup Conv
 * @{
 */

/**
 * @brief Convolution of floating-point sequences.
 * @param[in] *pSrcA points to the first input sequence.
 * @param[in] srcALen length of the first input sequence.
 * @param[in] *pSrcB points to the second input sequence.
 * @param[in] srcBLen length of the second input sequence.
 * @param[out] *pDst points to the location where the output result is written.  Length srcALen+srcBLen-1.
 * @return none.
 */

void xtensa_conv_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst)
{




  float32_t *pIn1 = pSrcA;                       /* inputA pointer */
  float32_t *pIn2 = pSrcB;                       /* inputB pointer */
  float32_t sum;                                 /* Accumulator */
  uint32_t i, j;                                 /* loop counters */

  /* Loop to calculate convolution for output length number of times */
  for (i = 0U; i < ((srcALen + srcBLen) - 1U); i++)
  {
    /* Initialize sum with zero to carry out MAC operations */
    sum = 0.0f;

    /* Loop to perform MAC operations according to convolution equation */
    for (j = 0U; j <= i; j++)
    {
      /* Check the array limitations */
      if ((((i - j) < srcBLen) && (j < srcALen)))
      {
        /* z[i] += x[i-j] * y[j] */
        sum += pIn1[j] * pIn2[i - j];
      }
    }
    /* Store the output in the destination buffer */
    pDst[i] = sum;
  }


}

/**
 * @} end of Conv group
 */
