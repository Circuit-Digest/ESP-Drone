

#include "xtensa_math.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup cmplx_conj Complex Conjugate
 *
 * Conjugates the elements of a complex data vector.
 *
 * The <code>pSrc</code> points to the source data and
 * <code>pDst</code> points to the where the result should be written.
 * <code>numSamples</code> specifies the number of complex samples
 * and the data in each array is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * Each array has a total of <code>2*numSamples</code> values.
 * The underlying algorithm is used:
 *
 * <pre>
 * for(n=0; n<numSamples; n++) {
 *     pDst[(2*n)+0)] = pSrc[(2*n)+0];     // real part
 *     pDst[(2*n)+1)] = -pSrc[(2*n)+1];    // imag part
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup cmplx_conj
 * @{
 */

/**
 * @brief  Floating-point complex conjugate.
 * @param  *pSrc points to the input vector
 * @param  *pDst points to the output vector
 * @param  numSamples number of complex samples in each vector
 * @return none.
 */

void xtensa_cmplx_conj_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  uint32_t blkCnt;                               /* loop counter */


  blkCnt = numSamples;


  while (blkCnt > 0U)
  {
    /* realOut + j (imagOut) = realIn + j (-1) imagIn */
    /* Calculate Complex Conjugate and then store the results in the destination buffer. */
    *pDst++ = *pSrc++;
    *pDst++ = -*pSrc++;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of cmplx_conj group
 */
