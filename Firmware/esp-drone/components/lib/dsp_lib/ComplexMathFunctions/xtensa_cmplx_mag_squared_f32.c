

#include "xtensa_math.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup cmplx_mag_squared Complex Magnitude Squared
 *
 * Computes the magnitude squared of the elements of a complex data vector.
 *
 * The <code>pSrc</code> points to the source data and
 * <code>pDst</code> points to the where the result should be written.
 * <code>numSamples</code> specifies the number of complex samples
 * in the input array and the data is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * The input array has a total of <code>2*numSamples</code> values;
 * the output array has a total of <code>numSamples</code> values.
 *
 * The underlying algorithm is used:
 *
 * <pre>
 * for(n=0; n<numSamples; n++) {
 *     pDst[n] = pSrc[(2*n)+0]^2 + pSrc[(2*n)+1]^2;
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup cmplx_mag_squared
 * @{
 */


/**
 * @brief  Floating-point complex magnitude squared
 * @param[in]  *pSrc points to the complex input vector
 * @param[out]  *pDst points to the real output vector
 * @param[in]  numSamples number of complex samples in the input vector
 * @return none.
 */

void xtensa_cmplx_mag_squared_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t real, imag;                          /* Temporary variables to store real and imaginary values */
  uint32_t blkCnt;                               /* loop counter */

  blkCnt = numSamples;
  while (blkCnt > 0U)
  {
    /* C[0] = (A[0] * A[0] + A[1] * A[1]) */
    real = *pSrc++;
    imag = *pSrc++;

    /* out = (real * real) + (imag * imag) */
    /* store the result in the destination buffer. */
    *pDst++ = (real * real) + (imag * imag);

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of cmplx_mag_squared group
 */
