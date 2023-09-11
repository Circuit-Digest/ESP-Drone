

#include "xtensa_math.h"
#include <math.h>

/**
 * @ingroup groupMath
 */

/**
 * @defgroup BasicAbs Vector Absolute Value
 *
 * Computes the absolute value of a vector on an element-by-element basis.
 *
 * <pre>
 *     pDst[n] = abs(pSrc[n]),   0 <= n < blockSize.
 * </pre>
 *
 * The functions support in-place computation allowing the source and
 * destination pointers to reference the same memory buffer.
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @addtogroup BasicAbs
 * @{
 */

/**
 * @brief Floating-point vector absolute value.
 * @param[in]       *pSrc points to the input buffer
 * @param[out]      *pDst points to the output buffer
 * @param[in]       blockSize number of samples in each vector
 * @return none.
 */

void xtensa_abs_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */


  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* C = |A| */
    /* Calculate absolute and then store the results in the destination buffer. */
    *pDst++ = fabsf(*pSrc++);

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of BasicAbs group
 */
