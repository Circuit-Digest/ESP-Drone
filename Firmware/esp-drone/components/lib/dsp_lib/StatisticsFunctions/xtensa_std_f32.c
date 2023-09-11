/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_std_f32.c
 * Description:  Standard deviation of the elements of a floating-point vector
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
 * @ingroup groupStats
 */

/**
 * @defgroup STD Standard deviation
 *
 * Calculates the standard deviation of the elements in the input vector.
 * The underlying algorithm is used:
 *
 * <pre>
 *   Result = sqrt((sumOfSquares - sum<sup>2</sup> / blockSize) / (blockSize - 1))
 *
 *     where, sumOfSquares = pSrc[0] * pSrc[0] + pSrc[1] * pSrc[1] + ... + pSrc[blockSize-1] * pSrc[blockSize-1]
 *
 *                     sum = pSrc[0] + pSrc[1] + pSrc[2] + ... + pSrc[blockSize-1]
 * </pre>
 *
 * There are separate functions for floating point, Q31, and Q15 data types.
 */

/**
 * @addtogroup STD
 * @{
 */


/**
 * @brief Standard deviation of the elements of a floating-point vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult standard deviation value returned here
 * @return none.
 */

void xtensa_std_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult)
{
  float32_t sum = 0.0f;                          /* Temporary result storage */
  float32_t sumOfSquares = 0.0f;                 /* Sum of squares */
  float32_t in;                                  /* input value */
  uint32_t blkCnt;                               /* loop counter */

  float32_t squareOfSum;                         /* Square of Sum */
  float32_t var;                                 /* Temporary varaince storage */


  if (blockSize == 1U)
  {
    *pResult = 0;
    return;
  }



  /* Loop over blockSize number of values */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sumOfSquares. */
    in = *pSrc++;
    sumOfSquares += in * in;

    /* C = (A[0] + A[1] + ... + A[blockSize-1]) */
    /* Compute Sum of the input samples
     * and then store the result in a temporary variable, sum. */
    sum += in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute the square of sum */
  squareOfSum = ((sum * sum) / (float32_t) blockSize);

  /* Compute the variance */
  var = ((sumOfSquares - squareOfSum) / (float32_t) (blockSize - 1.0f));

  /* Compute standard deviation and then store the result to the destination */
  xtensa_sqrt_f32(var, pResult);

}

/**
 * @} end of STD group
 */
