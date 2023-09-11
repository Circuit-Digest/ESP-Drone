

#include "xtensa_math.h"

/**
 * @ingroup groupMath
 */

/**
 * @defgroup scale Vector Scale
 *
 * Multiply a vector by a scalar value.  For floating-point data, the algorithm used is:
 *
 * <pre>
 *     pDst[n] = pSrc[n] * scale,   0 <= n < blockSize.
 * </pre>
 *
 * In the fixed-point Q7, Q15, and Q31 functions, <code>scale</code> is represented by
 * a fractional multiplication <code>scaleFract</code> and an arithmetic shift <code>shift</code>.
 * The shift allows the gain of the scaling operation to exceed 1.0.
 * The algorithm used with fixed-point data is:
 *
 * <pre>
 *     pDst[n] = (pSrc[n] * scaleFract) << shift,   0 <= n < blockSize.
 * </pre>
 *
 * The overall scale factor applied to the fixed-point data is
 * <pre>
 *     scale = scaleFract * 2^shift.
 * </pre>
 *
 * The functions support in-place computation allowing the source and destination
 * pointers to reference the same memory buffer.
 */

/**
 * @addtogroup scale
 * @{
 */

/**
 * @brief Multiplies a floating-point vector by a scalar.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       scale scale factor to be applied
 * @param[out]      *pDst points to the output vector
 * @param[in]       blockSize number of samples in the vector
 * @return none.
 */


void xtensa_scale_f32(
  float32_t * pSrc,
  float32_t scale,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */



  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;


  while (blkCnt > 0U)
  {
    /* C = A * scale */
    /* Scale the input and then store the result in the destination buffer. */
    *pDst++ = (*pSrc++) * scale;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of scale group
 */
