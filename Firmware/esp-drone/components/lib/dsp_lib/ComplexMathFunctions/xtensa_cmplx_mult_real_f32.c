

#include "xtensa_math.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup CmplxByRealMult Complex-by-Real Multiplication
 *
 * Multiplies a complex vector by a real vector and generates a complex result.
 * The data in the complex arrays is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * The parameter <code>numSamples</code> represents the number of complex
 * samples processed.  The complex arrays have a total of <code>2*numSamples</code>
 * real values while the real array has a total of <code>numSamples</code>
 * real values.
 *
 * The underlying algorithm is used:
 *
 * <pre>
 * for(n=0; n<numSamples; n++) {
 *     pCmplxDst[(2*n)+0] = pSrcCmplx[(2*n)+0] * pSrcReal[n];
 *     pCmplxDst[(2*n)+1] = pSrcCmplx[(2*n)+1] * pSrcReal[n];
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup CmplxByRealMult
 * @{
 */


/**
 * @brief  Floating-point complex-by-real multiplication
 * @param[in]  *pSrcCmplx points to the complex input vector
 * @param[in]  *pSrcReal points to the real input vector
 * @param[out]  *pCmplxDst points to the complex output vector
 * @param[in]  numSamples number of samples in each vector
 * @return none.
 */

void xtensa_cmplx_mult_real_f32(
  float32_t * pSrcCmplx,
  float32_t * pSrcReal,
  float32_t * pCmplxDst,
  uint32_t numSamples)
{
  float32_t in;                                  /* Temporary variable to store input value */
  uint32_t blkCnt;                               /* loop counters */


  blkCnt = numSamples;


  while (blkCnt > 0U)
  {
    /* C[2 * i] = A[2 * i] * B[i].            */
    /* C[2 * i + 1] = A[2 * i + 1] * B[i].        */
    in = *pSrcReal++;
    /* store the result in the destination buffer. */
    *pCmplxDst++ = (*pSrcCmplx++) * (in);
    *pCmplxDst++ = (*pSrcCmplx++) * (in);

    /* Decrement the numSamples loop counter */
    blkCnt--;
  }
}

/**
 * @} end of CmplxByRealMult group
 */
