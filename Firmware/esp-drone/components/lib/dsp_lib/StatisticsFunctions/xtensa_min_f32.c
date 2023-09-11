/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_min_f32.c
 * Description:  Minimum value of a floating-point vector
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
 * @defgroup Min Minimum
 *
 * Computes the minimum value of an array of data.
 * The function returns both the minimum value and its position within the array.
 * There are separate functions for floating-point, Q31, Q15, and Q7 data types.
 */

/**
 * @addtogroup Min
 * @{
 */


/**
 * @brief Minimum value of a floating-point vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult minimum value returned here
 * @param[out]      *pIndex index of minimum value returned here
 * @return none.
 */

void xtensa_min_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex)
{


  float32_t minVal1, out;                        /* Temporary variables to store the output value. */
  uint32_t blkCnt, outIndex;                     /* loop counter */

  /* Initialise the index value to zero. */
  outIndex = 0U;
  /* Load first input value that act as reference value for comparision */
  out = *pSrc++;

  blkCnt = (blockSize - 1U);


  while (blkCnt > 0U)
  {
    /* Initialize minVal to the next consecutive values one by one */
    minVal1 = *pSrc++;

    /* compare for the minimum value */
    if (out > minVal1)
    {
      /* Update the minimum value and it's index */
      out = minVal1;
      outIndex = blockSize - blkCnt;
    }

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Store the minimum value and it's index into destination pointers */
  *pResult = out;
  *pIndex = outIndex;
}

/**
 * @} end of Min group
 */
