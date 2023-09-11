

#include "xtensa_math.h"

/**
 * @ingroup groupMath
 */

/**
 * @defgroup BasicAdd Vector Addition
 *
 * Element-by-element addition of two vectors.
 *
 * <pre>
 *     pDst[n] = pSrcA[n] + pSrcB[n],   0 <= n < blockSize.
 * </pre>
 *
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @addtogroup BasicAdd
 * @{
 */

/**
 * @brief Floating-point vector addition.
 * @param[in]       *pSrcA points to the first input vector
 * @param[in]       *pSrcB points to the second input vector
 * @param[out]      *pDst points to the output vector
 * @param[in]       blockSize number of samples in each vector
 * @return none.
 */

void xtensa_add_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */
  blkCnt = blockSize;


  while (blkCnt > 0U)
  {
    /* C = A + B */
    /* Add and then store the results in the destination buffer. */
    *pDst++ = (*pSrcA++) + (*pSrcB++);

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
 * @} end of BasicAdd group
 */
