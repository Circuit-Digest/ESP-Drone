
#include "xtensa_math.h"

/**
 * @ingroup groupMath
 */

/**
 * @defgroup BasicMult Vector Multiplication
 *
 * Element-by-element multiplication of two vectors.
 *
 * <pre>
 *     pDst[n] = pSrcA[n] * pSrcB[n],   0 <= n < blockSize.
 * </pre>
 *
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @addtogroup BasicMult
 * @{
 */

/**
 * @brief Floating-point vector multiplication.
 * @param[in]       *pSrcA points to the first input vector
 * @param[in]       *pSrcB points to the second input vector
 * @param[out]      *pDst points to the output vector
 * @param[in]       blockSize number of samples in each vector
 * @return none.
 */

void xtensa_mult_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counters */

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;


  while (blkCnt > 0U)
  {
    /* C = A * B */
    /* Multiply the inputs and store the results in output buffer */
    *pDst++ = (*pSrcA++) * (*pSrcB++);

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }
}

/**
 * @} end of BasicMult group
 */
