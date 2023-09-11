

#include "xtensa_math.h"

/**
 * @ingroup groupMath
 */

/**
 * @defgroup negate Vector Negate
 *
 * Negates the elements of a vector.
 *
 * <pre>
 *     pDst[n] = -pSrc[n],   0 <= n < blockSize.
 * </pre>
 *
 * The functions support in-place computation allowing the source and
 * destination pointers to reference the same memory buffer.
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @addtogroup negate
 * @{
 */

/**
 * @brief  Negates the elements of a floating-point vector.
 * @param[in]  *pSrc points to the input vector
 * @param[out]  *pDst points to the output vector
 * @param[in]  blockSize number of samples in the vector
 * @return none.
 */

void xtensa_negate_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */




  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* C = -A */
    /* Negate and then store the results in the destination buffer. */
    *pDst++ = -*pSrc++;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of negate group
 */
