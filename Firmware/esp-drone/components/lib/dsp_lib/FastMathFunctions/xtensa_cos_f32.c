

#include "xtensa_math.h"
#include "xtensa_common_tables.h"
/**
 * @ingroup groupFastMath
 */

/**
 * @defgroup cos Cosine
 *
 * Computes the trigonometric cosine function using a combination of table lookup
 * and linear interpolation.  There are separate functions for
 * Q15, Q31, and floating-point data types.
 * The input to the floating-point version is in radians and in the range [0 2*pi) while the
 * fixed-point Q15 and Q31 have a scaled input with the range
 * [0 +0.9999] mapping to [0 2*pi).  The fixed-point range is chosen so that a
 * value of 2*pi wraps around to 0.
 *
 * The implementation is based on table lookup using 256 values together with linear interpolation.
 * The steps used are:
 *  -# Calculation of the nearest integer table index
 *  -# Compute the fractional portion (fract) of the table index.
 *  -# The final result equals <code>(1.0f-fract)*a + fract*b;</code>
 *
 * where
 * <pre>
 *    b=Table[index+0];
 *    c=Table[index+1];
 * </pre>
 */

 /**
 * @addtogroup cos
 * @{
 */

/**
 * @brief  Fast approximation to the trigonometric cosine function for floating-point data.
 * @param[in] x input value in radians.
 * @return cos(x).
 */

float32_t xtensa_cos_f32(
  float32_t x)
{
  float32_t cosVal, fract, in;                   /* Temporary variables for input, output */
  uint16_t index;                                /* Index variable */
  float32_t a, b;                                /* Two nearest output values */
  int32_t n;
  float32_t findex;

  /* input x is in radians */
  /* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table */
  in = x * 0.159154943092f + 0.25f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if (in < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1] */
  in = in - (float32_t) n;

  /* Calculation of index of the table */
  findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
  index = ((uint16_t)findex) & 0x1ff;

  /* fractional value calculation */
  fract = findex - (float32_t) index;

  /* Read two nearest values of input value from the cos table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  cosVal = (1.0f-fract)*a + fract*b;

  /* Return the output value */
  return (cosVal);
}

/**
 * @} end of cos group
 */
