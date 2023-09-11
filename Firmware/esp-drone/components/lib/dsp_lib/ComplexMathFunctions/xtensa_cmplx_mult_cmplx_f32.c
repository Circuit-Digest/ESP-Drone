

#include "xtensa_math.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup CmplxByCmplxMult Complex-by-Complex Multiplication
 *
 * Multiplies a complex vector by another complex vector and generates a complex result.
 * The data in the complex arrays is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * The parameter <code>numSamples</code> represents the number of complex
 * samples processed.  The complex arrays have a total of <code>2*numSamples</code>
 * real values.
 *
 * The underlying algorithm is used:
 *
 * <pre>
 * for(n=0; n<numSamples; n++) {
 *     pDst[(2*n)+0] = pSrcA[(2*n)+0] * pSrcB[(2*n)+0] - pSrcA[(2*n)+1] * pSrcB[(2*n)+1];
 *     pDst[(2*n)+1] = pSrcA[(2*n)+0] * pSrcB[(2*n)+1] + pSrcA[(2*n)+1] * pSrcB[(2*n)+0];
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup CmplxByCmplxMult
 * @{
 */


/**
 * @brief  Floating-point complex-by-complex multiplication
 * @param[in]  *pSrcA points to the first input vector
 * @param[in]  *pSrcB points to the second input vector
 * @param[out]  *pDst  points to the output vector
 * @param[in]  numSamples number of complex samples in each vector
 * @return none.
 */

void xtensa_cmplx_mult_cmplx_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t a1, b1, c1, d1;                      /* Temporary variables to store real and imaginary values */
  uint32_t blkCnt;                               /* loop counters */

  blkCnt = numSamples;


  while (blkCnt > 0U)
  {
    /* C[2 * i] = A[2 * i] * B[2 * i] - A[2 * i + 1] * B[2 * i + 1].  */
    /* C[2 * i + 1] = A[2 * i] * B[2 * i + 1] + A[2 * i + 1] * B[2 * i].  */
    a1 = *pSrcA++;
    b1 = *pSrcA++;
    c1 = *pSrcB++;
    d1 = *pSrcB++;

    /* store the result in the destination buffer. */
    *pDst++ = (a1 * c1) - (b1 * d1);
    *pDst++ = (a1 * d1) + (b1 * c1);

    /* Decrement the numSamples loop counter */
    blkCnt--;
  }
}

/**
 * @} end of CmplxByCmplxMult group
 */
