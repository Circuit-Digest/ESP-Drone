

#include "xtensa_math.h"

/**
 * @ingroup groupMath
 */

/**
 * @defgroup offset Vector Offset
 *
 * Adds a constant offset to each element of a vector.
 *
 * <pre>
 *     pDst[n] = pSrc[n] + offset,   0 <= n < blockSize.
 * </pre>
 *
 * The functions support in-place computation allowing the source and
 * destination pointers to reference the same memory buffer.
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @addtogroup offset
 * @{
 */

/**
 * @brief  Adds a constant offset to a floating-point vector.
 * @param[in]  *pSrc points to the input vector
 * @param[in]  offset is the offset to be added
 * @param[out]  *pDst points to the output vector
 * @param[in]  blockSize number of samples in the vector
 * @return none.
 */


void xtensa_offset_f32(
  float32_t * pSrc,
  float32_t offset,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */


  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* C = A + offset */
    /* Add offset and then store the result in the destination buffer. */
    *pDst++ = (*pSrc++) + offset;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of offset group
 */
