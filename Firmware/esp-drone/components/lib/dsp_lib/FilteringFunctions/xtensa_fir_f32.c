/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_fir_f32.c
 * Description:  Floating-point FIR filter processing function
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
* @defgroup FIR Finite Impulse Response (FIR) Filters
*
* This set of functions implements Finite Impulse Response (FIR) filters
* for Q7, Q15, Q31, and floating-point data types.  Fast versions of Q15 and Q31 are also provided.
* The functions operate on blocks of input and output data and each call to the function processes
* <code>blockSize</code> samples through the filter.  <code>pSrc</code> and
* <code>pDst</code> points to input and output arrays containing <code>blockSize</code> values.
*
* \par Algorithm:
* The FIR filter algorithm is based upon a sequence of multiply-accumulate (MAC) operations.
* Each filter coefficient <code>b[n]</code> is multiplied by a state variable which equals a previous input sample <code>x[n]</code>.
* <pre>
*    y[n] = b[0] * x[n] + b[1] * x[n-1] + b[2] * x[n-2] + ...+ b[numTaps-1] * x[n-numTaps+1]
* </pre>
* \par
* \image html FIR.gif "Finite Impulse Response filter"
* \par
* <code>pCoeffs</code> points to a coefficient array of size <code>numTaps</code>.
* Coefficients are stored in time reversed order.
* \par
* <pre>
*    {b[numTaps-1], b[numTaps-2], b[N-2], ..., b[1], b[0]}
* </pre>
* \par
* <code>pState</code> points to a state array of size <code>numTaps + blockSize - 1</code>.
* Samples in the state buffer are stored in the following order.
* \par
* <pre>
*    {x[n-numTaps+1], x[n-numTaps], x[n-numTaps-1], x[n-numTaps-2]....x[0], x[1], ..., x[blockSize-1]}
* </pre>
* \par
* Note that the length of the state buffer exceeds the length of the coefficient array by <code>blockSize-1</code>.
* The increased state buffer length allows circular addressing, which is traditionally used in the FIR filters,
* to be avoided and yields a significant speed improvement.
* The state variables are updated after each block of data is processed; the coefficients are untouched.
* \par Instance Structure
* The coefficients and state variables for a filter are stored together in an instance data structure.
* A separate instance structure must be defined for each filter.
* Coefficient arrays may be shared among several instances while state variable arrays cannot be shared.
* There are separate instance structure declarations for each of the 4 supported data types.
*
* \par Initialization Functions
* There is also an associated initialization function for each data type.
* The initialization function performs the following operations:
* - Sets the values of the internal structure fields.
* - Zeros out the values in the state buffer.
* To do this manually without calling the init function, assign the follow subfields of the instance structure:
* numTaps, pCoeffs, pState. Also set all of the values in pState to zero.
*
* \par
* Use of the initialization function is optional.
* However, if the initialization function is used, then the instance structure cannot be placed into a const data section.
* To place an instance structure into a const data section, the instance structure must be manually initialized.
* Set the values in the state buffer to zeros before static initialization.
* The code below statically initializes each of the 4 different data type filter instance structures
* <pre>
*xtensa_fir_instance_f32 S = {numTaps, pState, pCoeffs};
*xtensa_fir_instance_q31 S = {numTaps, pState, pCoeffs};
*xtensa_fir_instance_q15 S = {numTaps, pState, pCoeffs};
*xtensa_fir_instance_q7 S =  {numTaps, pState, pCoeffs};
* </pre>
*
* where <code>numTaps</code> is the number of filter coefficients in the filter; <code>pState</code> is the address of the state buffer;
* <code>pCoeffs</code> is the address of the coefficient buffer.
*
* \par Fixed-Point Behavior
* Care must be taken when using the fixed-point versions of the FIR filter functions.
* In particular, the overflow and saturation behavior of the accumulator used in each function must be considered.
* Refer to the function specific documentation below for usage guidelines.
*/

/**
* @addtogroup FIR
* @{
*/

/**
*
* @param[in]  *S points to an instance of the floating-point FIR filter structure.
* @param[in]  *pSrc points to the block of input data.
* @param[out] *pDst points to the block of output data.
* @param[in]  blockSize number of samples to process per call.
* @return     none.
*
*/




void xtensa_fir_f32(
const xtensa_fir_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize)
{
   float32_t *pState = S->pState;                 /* State pointer */
   float32_t *pCoeffs = S->pCoeffs;               /* Coefficient pointer */
   float32_t *pStateCurnt;                        /* Points to the current sample of the state */
   float32_t *px, *pb;                            /* Temporary pointers for state and coefficient buffers */
   uint32_t numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
   uint32_t i, tapCnt, blkCnt;                    /* Loop counters */

   /* Run the below code for Cortex-M0 */

   float32_t acc;

   /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
   /* pStateCurnt points to the location where the new input data should be written */
   pStateCurnt = &(S->pState[(numTaps - 1U)]);

   /* Initialize blkCnt with blockSize */
   blkCnt = blockSize;

   while (blkCnt > 0U)
   {
      /* Copy one sample at a time into state buffer */
      *pStateCurnt++ = *pSrc++;

      /* Set the accumulator to zero */
      acc = 0.0f;

      /* Initialize state pointer */
      px = pState;

      /* Initialize Coefficient pointer */
      pb = pCoeffs;

      i = numTaps;

      /* Perform the multiply-accumulates */
      do
      {
         /* acc =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-2] * x[n-numTaps-2] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0] */
         acc += *px++ * *pb++;
         i--;

      } while (i > 0U);

      /* The result is store in the destination buffer. */
      *pDst++ = acc;

      /* Advance state pointer by 1 for the next sample */
      pState = pState + 1;

      blkCnt--;
   }

   /* Processing is complete.
   ** Now copy the last numTaps - 1 samples to the starting of the state buffer.
   ** This prepares the state buffer for the next function call. */

   /* Points to the start of the state buffer */
   pStateCurnt = S->pState;

   /* Copy numTaps number of values */
   tapCnt = numTaps - 1U;

   /* Copy data */
   while (tapCnt > 0U)
   {
      *pStateCurnt++ = *pState++;

      /* Decrement the loop counter */
      tapCnt--;
   }

}


/**
* @} end of FIR group
*/
