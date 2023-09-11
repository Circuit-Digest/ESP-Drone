

#include "xtensa_math.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup cmplx_mag Complex Magnitude
 *
 * Computes the magnitude of the elements of a complex data vector.
 *
 * The <code>pSrc</code> points to the source data and
 * <code>pDst</code> points to the where the result should be written.
 * <code>numSamples</code> specifies the number of complex samples
 * in the input array and the data is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * The input array has a total of <code>2*numSamples</code> values;
 * the output array has a total of <code>numSamples</code> values.
 * The underlying algorithm is used:
 *
 * <pre>
 * for(n=0; n<numSamples; n++) {
 *     pDst[n] = sqrt(pSrc[(2*n)+0]^2 + pSrc[(2*n)+1]^2);
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup cmplx_mag
 * @{
 */
/**
 * @brief Floating-point complex magnitude.
 * @param[in]       *pSrc points to complex input buffer
 * @param[out]      *pDst points to real output buffer
 * @param[in]       numSamples number of complex samples in the input vector
 * @return none.
 *
 */


void xtensa_cmplx_mag_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t realIn, imagIn;                      /* Temporary variables to hold input values */



  while (numSamples > 0U)
  {
    /* out = sqrt((real * real) + (imag * imag)) */
    realIn = *pSrc++;
    imagIn = *pSrc++;
    /* store the result in the destination buffer. */
    xtensa_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

    /* Decrement the loop counter */
    numSamples--;
  }


}

/**
 * @} end of cmplx_mag group
 */
