

#include "xtensa_math.h"

/**
 * @ingroup groupMath
 */

/**
 * @defgroup dot_prod Vector Dot Product
 *
 * Computes the dot product of two vectors.
 * The vectors are multiplied element-by-element and then summed.
 *
 * <pre>
 *     sum = pSrcA[0]*pSrcB[0] + pSrcA[1]*pSrcB[1] + ... + pSrcA[blockSize-1]*pSrcB[blockSize-1]
 * </pre>
 *
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @addtogroup dot_prod
 * @{
 */

/**
 * @brief Dot product of floating-point vectors.
 * @param[in]       *pSrcA points to the first input vector
 * @param[in]       *pSrcB points to the second input vector
 * @param[in]       blockSize number of samples in each vector
 * @param[out]      *result output result returned here
 * @return none.
 */


void xtensa_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t blockSize,
  float32_t * result)
{
  float32_t sum = 0.0f;                          /* Temporary result storage */
  uint32_t blkCnt;                               /* loop counter */




  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;



  while (blkCnt > 0U)
  {
    /* C = A[0]* B[0] + A[1]* B[1] + A[2]* B[2] + .....+ A[blockSize-1]* B[blockSize-1] */
    /* Calculate dot product and then store the result in a temporary buffer. */
    sum += (*pSrcA++) * (*pSrcB++);

    /* Decrement the loop counter */
    blkCnt--;
  }
  /* Store the result back in the destination buffer */
  *result = sum;
}

/**
 * @} end of dot_prod group
 */
