/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_mat_trans_f32.c
 * Description:  Floating-point matrix transpose
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

/**
 * @defgroup MatrixTrans Matrix Transpose
 *
 * Tranposes a matrix.
 * Transposing an <code>M x N</code> matrix flips it around the center diagonal and results in an <code>N x M</code> matrix.
 * \image html MatrixTranspose.gif "Transpose of a 3 x 3 matrix"
 */

#include "xtensa_math.h"

/**
 * @ingroup groupMatrix
 */

/**
 * @addtogroup MatrixTrans
 * @{
 */

/**
  * @brief Floating-point matrix transpose.
  * @param[in]  *pSrc points to the input matrix
  * @param[out] *pDst points to the output matrix
  * @return 	The function returns either  <code>XTENSA_MATH_SIZE_MISMATCH</code>
  * or <code>XTENSA_MATH_SUCCESS</code> based on the outcome of size checking.
  */


xtensa_status xtensa_mat_trans_f32(
  const xtensa_matrix_instance_f32 * pSrc,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn = pSrc->pData;                  /* input data matrix pointer */
  float32_t *pOut = pDst->pData;                 /* output data matrix pointer */
  float32_t *px;                                 /* Temporary output data matrix pointer */
  uint16_t nRows = pSrc->numRows;                /* number of rows */
  uint16_t nColumns = pSrc->numCols;             /* number of columns */



  uint16_t col, i = 0U, row = nRows;             /* loop counters */
  xtensa_status status;                             /* status of matrix transpose  */


#ifdef XTENSA_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if ((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as XTENSA_MATH_SIZE_MISMATCH */
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef XTENSA_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* Initialize column loop counter */
      col = nColumns;

      while (col > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        col--;
      }


      i++;

      /* Decrement the row loop counter */
      row--;

    } while (row > 0U);          /* row loop end  */

    /* Set status as XTENSA_MATH_SUCCESS */
    status = XTENSA_MATH_SUCCESS;
  }

  /* Return to application */
  return (status);
}

/**
 * @} end of MatrixTrans group
 */
