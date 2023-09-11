/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_iir_lattice_f32.c
 * Description:  Floating-point IIR Lattice filter processing function
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
 * @defgroup IIR_Lattice Infinite Impulse Response (IIR) Lattice Filters
 *
 * This set of functions implements lattice filters
 * for Q15, Q31 and floating-point data types.  Lattice filters are used in a
 * variety of adaptive filter applications.  The filter structure has feedforward and
 * feedback components and the net impulse response is infinite length.
 * The functions operate on blocks
 * of input and output data and each call to the function processes
 * <code>blockSize</code> samples through the filter.  <code>pSrc</code> and
 * <code>pDst</code> point to input and output arrays containing <code>blockSize</code> values.

 * \par Algorithm:
 * \image html IIRLattice.gif "Infinite Impulse Response Lattice filter"
 * <pre>
 *    fN(n)   =  x(n)
 *    fm-1(n) = fm(n) - km * gm-1(n-1)   for m = N, N-1, ...1
 *    gm(n)   = km * fm-1(n) + gm-1(n-1) for m = N, N-1, ...1
 *    y(n)    = vN * gN(n) + vN-1 * gN-1(n) + ...+ v0 * g0(n)
 * </pre>
 * \par
 * <code>pkCoeffs</code> points to array of reflection coefficients of size <code>numStages</code>.
 * Reflection coefficients are stored in time-reversed order.
 * \par
 * <pre>
 *    {kN, kN-1, ....k1}
 * </pre>
 * <code>pvCoeffs</code> points to the array of ladder coefficients of size <code>(numStages+1)</code>.
 * Ladder coefficients are stored in time-reversed order.
 * \par
 * <pre>
 *    {vN, vN-1, ...v0}
 * </pre>
 * <code>pState</code> points to a state array of size <code>numStages + blockSize</code>.
 * The state variables shown in the figure above (the g values) are stored in the <code>pState</code> array.
 * The state variables are updated after each block of data is processed; the coefficients are untouched.
 * \par Instance Structure
 * The coefficients and state variables for a filter are stored together in an instance data structure.
 * A separate instance structure must be defined for each filter.
 * Coefficient arrays may be shared among several instances while state variable arrays cannot be shared.
 * There are separate instance structure declarations for each of the 3 supported data types.
  *
 * \par Initialization Functions
 * There is also an associated initialization function for each data type.
 * The initialization function performs the following operations:
 * - Sets the values of the internal structure fields.
 * - Zeros out the values in the state buffer.
 * To do this manually without calling the init function, assign the follow subfields of the instance structure:
 * numStages, pkCoeffs, pvCoeffs, pState. Also set all of the values in pState to zero.
 *
 * \par
 * Use of the initialization function is optional.
 * However, if the initialization function is used, then the instance structure cannot be placed into a const data section.
 * To place an instance structure into a const data section, the instance structure must be manually initialized.
 * Set the values in the state buffer to zeros and then manually initialize the instance structure as follows:
 * <pre>
 *xtensa_iir_lattice_instance_f32 S = {numStages, pState, pkCoeffs, pvCoeffs};
 *xtensa_iir_lattice_instance_q31 S = {numStages, pState, pkCoeffs, pvCoeffs};
 *xtensa_iir_lattice_instance_q15 S = {numStages, pState, pkCoeffs, pvCoeffs};
 * </pre>
 * \par
 * where <code>numStages</code> is the number of stages in the filter; <code>pState</code> points to the state buffer array;
 * <code>pkCoeffs</code> points to array of the reflection coefficients; <code>pvCoeffs</code> points to the array of ladder coefficients.
 * \par Fixed-Point Behavior
 * Care must be taken when using the fixed-point versions of the IIR lattice filter functions.
 * In particular, the overflow and saturation behavior of the accumulator used in each function must be considered.
 * Refer to the function specific documentation below for usage guidelines.
 */

/**
 * @addtogroup IIR_Lattice
 * @{
 */

/**
 * @brief Processing function for the floating-point IIR lattice filter.
 * @param[in] *S points to an instance of the floating-point IIR lattice structure.
 * @param[in] *pSrc points to the block of input data.
 * @param[out] *pDst points to the block of output data.
 * @param[in] blockSize number of samples to process.
 * @return none.
 */



void xtensa_iir_lattice_f32(
  const xtensa_iir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  float32_t fcurr, fnext = 0, gcurr, gnext;      /* Temporary variables for lattice stages */
  float32_t acc;                                 /* Accumlator */
  uint32_t blkCnt, tapCnt;                       /* temporary variables for counts */
  float32_t *px1, *px2, *pk, *pv;                /* temporary pointers for state and coef */
  uint32_t numStages = S->numStages;             /* number of stages */
  float32_t *pState;                             /* State pointer */
  float32_t *pStateCurnt;                        /* State current pointer */


  /* Run the below code for Cortex-M0 */

  blkCnt = blockSize;

  pState = &S->pState[0];

  /* Sample processing */
  while (blkCnt > 0U)
  {
    /* Read Sample from input buffer */
    /* fN(n) = x(n) */
    fcurr = *pSrc++;

    /* Initialize state read pointer */
    px1 = pState;
    /* Initialize state write pointer */
    px2 = pState;
    /* Set accumulator to zero */
    acc = 0.0f;
    /* Initialize Ladder coeff pointer */
    pv = &S->pvCoeffs[0];
    /* Initialize Reflection coeff pointer */
    pk = &S->pkCoeffs[0];


    /* Process sample for numStages */
    tapCnt = numStages;

    while (tapCnt > 0U)
    {
      gcurr = *px1++;
      /* Process sample for last taps */
      fnext = fcurr - ((*pk) * gcurr);
      gnext = (fnext * (*pk++)) + gcurr;

      /* Output samples for last taps */
      acc += (gnext * (*pv++));
      *px2++ = gnext;
      fcurr = fnext;

      /* Decrementing loop counter */
      tapCnt--;

    }

    /* y(n) += g0(n) * v0 */
    acc += (fnext * (*pv));

    *px2++ = fnext;

    /* write out into pDst */
    *pDst++ = acc;

    /* Advance the state pointer by 1 to process the next group of samples */
    pState = pState + 1U;
    blkCnt--;

  }

  /* Processing is complete. Now copy last S->numStages samples to start of the buffer
     for the preperation of next frame process */

  /* Points to the start of the state buffer */
  pStateCurnt = &S->pState[0];
  pState = &S->pState[blockSize];

  tapCnt = numStages;

  /* Copy the data */
  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;

    /* Decrement the loop counter */
    tapCnt--;
  }

}



/**
 * @} end of IIR_Lattice group
 */
