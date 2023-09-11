/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_mean_f32.c
 * Description:  Mean value of a floating-point vector
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
 * @defgroup mean Mean
 *
 * Calculates the mean of the input vector. Mean is defined as the average of the elements in the vector.
 * The underlying algorithm is used:
 *
 * <pre>
 * 	Result = (pSrc[0] + pSrc[1] + pSrc[2] + ... + pSrc[blockSize-1]) / blockSize;
 * </pre>
 *
 * There are separate functions for floating-point, Q31, Q15, and Q7 data types.
 */

/**
 * @addtogroup mean
 * @{
 */


/**
 * @brief Mean value of a floating-point vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult mean value returned here
 * @return none.
 */

void xtensa_mean_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult)
{
  float32_t sum = 0.0f;                          /* Temporary result storage */
  uint32_t blkCnt;                               /* loop counter */



  /* Loop over blockSize number of values */
  blkCnt = blockSize;


  while (blkCnt > 0U)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    sum += *pSrc++;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
  /* Store the result to the destination */
  *pResult = sum / (float32_t) blockSize;
}

/**
 * @} end of mean group
 */
