/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_conv_partial_f32.c
 * Description:  Partial convolution of floating-point sequences
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

#include "xtensa_math.h"

/**
 * @ingroup groupFilters
 */

/**
 * @defgroup PartialConv Partial Convolution
 *
 * Partial Convolution is equivalent to Convolution except that a subset of the output samples is generated.
 * Each function has two additional arguments.
 * <code>firstIndex</code> specifies the starting index of the subset of output samples.
 * <code>numPoints</code> is the number of output samples to compute.
 * The function computes the output in the range
 * <code>[firstIndex, ..., firstIndex+numPoints-1]</code>.
 * The output array <code>pDst</code> contains <code>numPoints</code> values.
 *
 * The allowable range of output indices is [0 srcALen+srcBLen-2].
 * If the requested subset does not fall in this range then the functions return XTENSA_MATH_ARGUMENT_ERROR.
 * Otherwise the functions return XTENSA_MATH_SUCCESS.
 * \note Refer xtensa_conv_f32() for details on fixed point behavior.
 *
 *
 * <b>Fast Versions</b>
 *
 * \par
 * Fast versions are supported for Q31 and Q15 of partial convolution.  Cycles for Fast versions are less compared to Q31 and Q15 of partial conv and the design requires
 * the input signals should be scaled down to avoid intermediate overflows.
 *
 *
 * <b>Opt Versions</b>
 *
 * \par
 * Opt versions are supported for Q15 and Q7.  Design uses internal scratch buffer for getting good optimisation.
 * These versions are optimised in cycles and consumes more memory(Scratch memory) compared to Q15 and Q7 versions of partial convolution
 */

/**
 * @addtogroup PartialConv
 * @{
 */

/**
 * @brief Partial convolution of floating-point sequences.
 * @param[in]       *pSrcA points to the first input sequence.
 * @param[in]       srcALen length of the first input sequence.
 * @param[in]       *pSrcB points to the second input sequence.
 * @param[in]       srcBLen length of the second input sequence.
 * @param[out]      *pDst points to the location where the output result is written.
 * @param[in]       firstIndex is the first output sample to start with.
 * @param[in]       numPoints is the number of output points to be computed.
 * @return  Returns either XTENSA_MATH_SUCCESS if the function completed correctly or XTENSA_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
 */

xtensa_status xtensa_conv_partial_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints)
{




  float32_t *pIn1 = pSrcA;                       /* inputA pointer */
  float32_t *pIn2 = pSrcB;                       /* inputB pointer */
  float32_t sum;                                 /* Accumulator */
  uint32_t i, j;                                 /* loop counters */
  xtensa_status status;                             /* status of Partial convolution */

  /* Check for range of output samples to be calculated */
  if ((firstIndex + numPoints) > ((srcALen + (srcBLen - 1U))))
  {
    /* Set status as XTENSA_ARGUMENT_ERROR */
    status = XTENSA_MATH_ARGUMENT_ERROR;
  }
  else
  {
    /* Loop to calculate convolution for output length number of values */
    for (i = firstIndex; i <= (firstIndex + numPoints - 1); i++)
    {
      /* Initialize sum with zero to carry on MAC operations */
      sum = 0.0f;

      /* Loop to perform MAC operations according to convolution equation */
      for (j = 0U; j <= i; j++)
      {
        /* Check the array limitations for inputs */
        if ((((i - j) < srcBLen) && (j < srcALen)))
        {
          /* z[i] += x[i-j] * y[j] */
          sum += pIn1[j] * pIn2[i - j];
        }
      }
      /* Store the output in the destination buffer */
      pDst[i] = sum;
    }
    /* set status as XTENSA_SUCCESS as there are no argument errors */
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);

}

/**
 * @} end of PartialConv group
 */
