/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_mat_sub_f32.c
 * Description:  Floating-point matrix subtraction
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
 * @ingroup groupMatrix
 */

/**
 * @defgroup MatrixSub Matrix Subtraction
 *
 * Subtract two matrices.
 * \image html MatrixSubtraction.gif "Subraction of two 3 x 3 matrices"
 *
 * The functions check to make sure that
 * <code>pSrcA</code>, <code>pSrcB</code>, and <code>pDst</code> have the same
 * number of rows and columns.
 */

/**
 * @addtogroup MatrixSub
 * @{
 */

/**
 * @brief Floating-point matrix subtraction
 * @param[in]       *pSrcA points to the first input matrix structure
 * @param[in]       *pSrcB points to the second input matrix structure
 * @param[out]      *pDst points to output matrix structure
 * @return     		The function returns either
 * <code>XTENSA_MATH_SIZE_MISMATCH</code> or <code>XTENSA_MATH_SUCCESS</code> based on the outcome of size checking.
 */

xtensa_status xtensa_mat_sub_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
  float32_t *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
  float32_t *pOut = pDst->pData;                 /* output data matrix pointer  */


  uint32_t numSamples;                           /* total number of elements in the matrix  */
  uint32_t blkCnt;                               /* loop counters */
  xtensa_status status;                             /* status of matrix subtraction */

#ifdef XTENSA_MATH_MATRIX_CHECK
  /* Check for matrix mismatch condition */
  if ((pSrcA->numRows != pSrcB->numRows) ||
     (pSrcA->numCols != pSrcB->numCols) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
  {
    /* Set status as XTENSA_MATH_SIZE_MISMATCH */
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef XTENSA_MATH_MATRIX_CHECK    */
  {
    /* Total number of samples in the input matrix */
    numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;



    /* Initialize blkCnt with number of samples */
    blkCnt = numSamples;


    while (blkCnt > 0U)
    {
      /* C(m,n) = A(m,n) - B(m,n) */
      /* Subtract and then store the results in the destination buffer. */
      *pOut++ = (*pIn1++) - (*pIn2++);

      /* Decrement the loop counter */
      blkCnt--;
    }

    /* Set status as XTENSA_MATH_SUCCESS */
    status = XTENSA_MATH_SUCCESS;
  }

  /* Return to application */
  return (status);
}

/**
 * @} end of MatrixSub group
 */
