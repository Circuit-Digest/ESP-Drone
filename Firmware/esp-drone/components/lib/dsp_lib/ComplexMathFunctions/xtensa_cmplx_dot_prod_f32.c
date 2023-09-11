

#include "xtensa_math.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup cmplx_dot_prod Complex Dot Product
 *
 * Computes the dot product of two complex vectors.
 * The vectors are multiplied element-by-element and then summed.
 *
 * The <code>pSrcA</code> points to the first complex input vector and
 * <code>pSrcB</code> points to the second complex input vector.
 * <code>numSamples</code> specifies the number of complex samples
 * and the data in each array is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * Each array has a total of <code>2*numSamples</code> values.
 *
 * The underlying algorithm is used:
 * <pre>
 * realResult=0;
 * imagResult=0;
 * for(n=0; n<numSamples; n++) {
 *     realResult += pSrcA[(2*n)+0]*pSrcB[(2*n)+0] - pSrcA[(2*n)+1]*pSrcB[(2*n)+1];
 *     imagResult += pSrcA[(2*n)+0]*pSrcB[(2*n)+1] + pSrcA[(2*n)+1]*pSrcB[(2*n)+0];
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup cmplx_dot_prod
 * @{
 */

/**
 * @brief  Floating-point complex dot product
 * @param  *pSrcA points to the first input vector
 * @param  *pSrcB points to the second input vector
 * @param  numSamples number of complex samples in each vector
 * @param  *realResult real part of the result returned here
 * @param  *imagResult imaginary part of the result returned here
 * @return none.
 */

void xtensa_cmplx_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t numSamples,
  float32_t * realResult,
  float32_t * imagResult)
{
  float32_t real_sum = 0.0f, imag_sum = 0.0f;    /* Temporary result storage */
  float32_t a0,b0,c0,d0;




  while (numSamples > 0U)
  {
      a0 = *pSrcA++;
      b0 = *pSrcA++;
      c0 = *pSrcB++;
      d0 = *pSrcB++;

      real_sum += a0 * c0;
      imag_sum += a0 * d0;
      real_sum -= b0 * d0;
      imag_sum += b0 * c0;

      /* Decrement the loop counter */
      numSamples--;
  }


  /* Store the real and imaginary results in the destination buffers */
  *realResult = real_sum;
  *imagResult = imag_sum;
}

/**
 * @} end of cmplx_dot_prod group
 */
