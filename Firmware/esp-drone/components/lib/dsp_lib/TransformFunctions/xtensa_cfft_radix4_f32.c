/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        xtensa_cfft_radix4_f32.c
 * Description:  Radix-4 Decimation in Frequency CFFT & CIFFT Floating point processing function
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 XTENSA Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "xtensa_math.h"

extern void xtensa_bitreversal_f32(
float32_t * pSrc,
uint16_t fftSize,
uint16_t bitRevFactor,
uint16_t * pBitRevTab);

void xtensa_radix4_butterfly_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier);

void xtensa_radix4_butterfly_inverse_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier,
float32_t onebyfftLen);


/**
* @ingroup groupTransforms
*/

/**
* @addtogroup ComplexFFT
* @{
*/

/**
* @details
* @brief Processing function for the floating-point Radix-4 CFFT/CIFFT.
* @deprecated Do not use this function.  It has been superseded by \ref xtensa_cfft_f32 and will be removed
* in the future.
* @param[in]      *S    points to an instance of the floating-point Radix-4 CFFT/CIFFT structure.
* @param[in, out] *pSrc points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place.
* @return none.
*/

void xtensa_cfft_radix4_f32(
  const xtensa_cfft_radix4_instance_f32 * S,
  float32_t * pSrc)
{
   if (S->ifftFlag == 1U)
   {
      /*  Complex IFFT radix-4  */
      xtensa_radix4_butterfly_inverse_f32(pSrc, S->fftLen, S->pTwiddle, S->twidCoefModifier, S->onebyfftLen);
   }
   else
   {
      /*  Complex FFT radix-4  */
      xtensa_radix4_butterfly_f32(pSrc, S->fftLen, S->pTwiddle, S->twidCoefModifier);
   }

   if (S->bitReverseFlag == 1U)
   {
      /*  Bit Reversal */
      xtensa_bitreversal_f32(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
   }

}

/**
* @} end of ComplexFFT group
*/

/* ----------------------------------------------------------------------
 * Internal helper function used by the FFTs
 * ---------------------------------------------------------------------- */

/*
* @brief  Core function for the floating-point CFFT butterfly process.
* @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.
* @param[in]      fftLen           length of the FFT.
* @param[in]      *pCoef           points to the twiddle coefficient buffer.
* @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.
* @return none.
*/

void xtensa_radix4_butterfly_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier)
{

   float32_t co1, co2, co3, si1, si2, si3;
   uint32_t ia1, ia2, ia3;
   uint32_t i0, i1, i2, i3;
   uint32_t n1, n2, j, k;



   float32_t t1, t2, r1, r2, s1, s2;

   /* Run the below code for Cortex-M0 */

   /*  Initializations for the fft calculation */
   n2 = fftLen;
   n1 = n2;
   for (k = fftLen; k > 1U; k >>= 2U)
   {
      /*  Initializations for the fft calculation */
      n1 = n2;
      n2 >>= 2U;
      ia1 = 0U;

      /*  FFT Calculation */
      j = 0;
      do
      {
         /*  index calculation for the coefficients */
         ia2 = ia1 + ia1;
         ia3 = ia2 + ia1;
         co1 = pCoef[ia1 * 2U];
         si1 = pCoef[(ia1 * 2U) + 1U];
         co2 = pCoef[ia2 * 2U];
         si2 = pCoef[(ia2 * 2U) + 1U];
         co3 = pCoef[ia3 * 2U];
         si3 = pCoef[(ia3 * 2U) + 1U];

         /*  Twiddle coefficients index modifier */
         ia1 = ia1 + twidCoefModifier;

         i0 = j;
         do
         {
            /*  index calculation for the input as, */
            /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
            i1 = i0 + n2;
            i2 = i1 + n2;
            i3 = i2 + n2;

            /* xa + xc */
            r1 = pSrc[(2U * i0)] + pSrc[(2U * i2)];

            /* xa - xc */
            r2 = pSrc[(2U * i0)] - pSrc[(2U * i2)];

            /* ya + yc */
            s1 = pSrc[(2U * i0) + 1U] + pSrc[(2U * i2) + 1U];

            /* ya - yc */
            s2 = pSrc[(2U * i0) + 1U] - pSrc[(2U * i2) + 1U];

            /* xb + xd */
            t1 = pSrc[2U * i1] + pSrc[2U * i3];

            /* xa' = xa + xb + xc + xd */
            pSrc[2U * i0] = r1 + t1;

            /* xa + xc -(xb + xd) */
            r1 = r1 - t1;

            /* yb + yd */
            t2 = pSrc[(2U * i1) + 1U] + pSrc[(2U * i3) + 1U];

            /* ya' = ya + yb + yc + yd */
            pSrc[(2U * i0) + 1U] = s1 + t2;

            /* (ya + yc) - (yb + yd) */
            s1 = s1 - t2;

            /* (yb - yd) */
            t1 = pSrc[(2U * i1) + 1U] - pSrc[(2U * i3) + 1U];

            /* (xb - xd) */
            t2 = pSrc[2U * i1] - pSrc[2U * i3];

            /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
            pSrc[2U * i1] = (r1 * co2) + (s1 * si2);

            /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
            pSrc[(2U * i1) + 1U] = (s1 * co2) - (r1 * si2);

            /* (xa - xc) + (yb - yd) */
            r1 = r2 + t1;

            /* (xa - xc) - (yb - yd) */
            r2 = r2 - t1;

            /* (ya - yc) -  (xb - xd) */
            s1 = s2 - t2;

            /* (ya - yc) +  (xb - xd) */
            s2 = s2 + t2;

            /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
            pSrc[2U * i2] = (r1 * co1) + (s1 * si1);

            /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
            pSrc[(2U * i2) + 1U] = (s1 * co1) - (r1 * si1);

            /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
            pSrc[2U * i3] = (r2 * co3) + (s2 * si3);

            /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
            pSrc[(2U * i3) + 1U] = (s2 * co3) - (r2 * si3);

            i0 += n1;
         } while ( i0 < fftLen);
         j++;
      } while (j <= (n2 - 1U));
      twidCoefModifier <<= 2U;
   }


}

/*
* @brief  Core function for the floating-point CIFFT butterfly process.
* @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.
* @param[in]      fftLen           length of the FFT.
* @param[in]      *pCoef           points to twiddle coefficient buffer.
* @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.
* @param[in]      onebyfftLen      value of 1/fftLen.
* @return none.
*/

void xtensa_radix4_butterfly_inverse_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier,
float32_t onebyfftLen)
{
   float32_t co1, co2, co3, si1, si2, si3;
   uint32_t ia1, ia2, ia3;
   uint32_t i0, i1, i2, i3;
   uint32_t n1, n2, j, k;


   float32_t t1, t2, r1, r2, s1, s2;

   /* Run the below code for Cortex-M0 */

   /*  Initializations for the first stage */
   n2 = fftLen;
   n1 = n2;

   /*  Calculation of first stage */
   for (k = fftLen; k > 4U; k >>= 2U)
   {
      /*  Initializations for the first stage */
      n1 = n2;
      n2 >>= 2U;
      ia1 = 0U;

      /*  Calculation of first stage */
      j = 0;
      do
      {
         /*  index calculation for the coefficients */
         ia2 = ia1 + ia1;
         ia3 = ia2 + ia1;
         co1 = pCoef[ia1 * 2U];
         si1 = pCoef[(ia1 * 2U) + 1U];
         co2 = pCoef[ia2 * 2U];
         si2 = pCoef[(ia2 * 2U) + 1U];
         co3 = pCoef[ia3 * 2U];
         si3 = pCoef[(ia3 * 2U) + 1U];

         /*  Twiddle coefficients index modifier */
         ia1 = ia1 + twidCoefModifier;

         i0 = j;
         do
         {
            /*  index calculation for the input as, */
            /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
            i1 = i0 + n2;
            i2 = i1 + n2;
            i3 = i2 + n2;

            /* xa + xc */
            r1 = pSrc[(2U * i0)] + pSrc[(2U * i2)];

            /* xa - xc */
            r2 = pSrc[(2U * i0)] - pSrc[(2U * i2)];

            /* ya + yc */
            s1 = pSrc[(2U * i0) + 1U] + pSrc[(2U * i2) + 1U];

            /* ya - yc */
            s2 = pSrc[(2U * i0) + 1U] - pSrc[(2U * i2) + 1U];

            /* xb + xd */
            t1 = pSrc[2U * i1] + pSrc[2U * i3];

            /* xa' = xa + xb + xc + xd */
            pSrc[2U * i0] = r1 + t1;

            /* xa + xc -(xb + xd) */
            r1 = r1 - t1;

            /* yb + yd */
            t2 = pSrc[(2U * i1) + 1U] + pSrc[(2U * i3) + 1U];

            /* ya' = ya + yb + yc + yd */
            pSrc[(2U * i0) + 1U] = s1 + t2;

            /* (ya + yc) - (yb + yd) */
            s1 = s1 - t2;

            /* (yb - yd) */
            t1 = pSrc[(2U * i1) + 1U] - pSrc[(2U * i3) + 1U];

            /* (xb - xd) */
            t2 = pSrc[2U * i1] - pSrc[2U * i3];

            /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
            pSrc[2U * i1] = (r1 * co2) - (s1 * si2);

            /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
            pSrc[(2U * i1) + 1U] = (s1 * co2) + (r1 * si2);

            /* (xa - xc) - (yb - yd) */
            r1 = r2 - t1;

            /* (xa - xc) + (yb - yd) */
            r2 = r2 + t1;

            /* (ya - yc) +  (xb - xd) */
            s1 = s2 + t2;

            /* (ya - yc) -  (xb - xd) */
            s2 = s2 - t2;

            /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
            pSrc[2U * i2] = (r1 * co1) - (s1 * si1);

            /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
            pSrc[(2U * i2) + 1U] = (s1 * co1) + (r1 * si1);

            /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
            pSrc[2U * i3] = (r2 * co3) - (s2 * si3);

            /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
            pSrc[(2U * i3) + 1U] = (s2 * co3) + (r2 * si3);

            i0 += n1;
         } while ( i0 < fftLen);
         j++;
      } while (j <= (n2 - 1U));
      twidCoefModifier <<= 2U;
   }
   /*  Initializations of last stage */
   n1 = n2;
   n2 >>= 2U;

   /*  Calculations of last stage */
   for (i0 = 0U; i0 <= (fftLen - n1); i0 += n1)
   {
      /*  index calculation for the input as, */
      /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
      i1 = i0 + n2;
      i2 = i1 + n2;
      i3 = i2 + n2;

      /*  Butterfly implementation */
      /* xa + xc */
      r1 = pSrc[2U * i0] + pSrc[2U * i2];

      /* xa - xc */
      r2 = pSrc[2U * i0] - pSrc[2U * i2];

      /* ya + yc */
      s1 = pSrc[(2U * i0) + 1U] + pSrc[(2U * i2) + 1U];

      /* ya - yc */
      s2 = pSrc[(2U * i0) + 1U] - pSrc[(2U * i2) + 1U];

      /* xc + xd */
      t1 = pSrc[2U * i1] + pSrc[2U * i3];

      /* xa' = xa + xb + xc + xd */
      pSrc[2U * i0] = (r1 + t1) * onebyfftLen;

      /* (xa + xb) - (xc + xd) */
      r1 = r1 - t1;

      /* yb + yd */
      t2 = pSrc[(2U * i1) + 1U] + pSrc[(2U * i3) + 1U];

      /* ya' = ya + yb + yc + yd */
      pSrc[(2U * i0) + 1U] = (s1 + t2) * onebyfftLen;

      /* (ya + yc) - (yb + yd) */
      s1 = s1 - t2;

      /* (yb-yd) */
      t1 = pSrc[(2U * i1) + 1U] - pSrc[(2U * i3) + 1U];

      /* (xb-xd) */
      t2 = pSrc[2U * i1] - pSrc[2U * i3];

      /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
      pSrc[2U * i1] = r1 * onebyfftLen;

      /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
      pSrc[(2U * i1) + 1U] = s1 * onebyfftLen;

      /* (xa - xc) - (yb-yd) */
      r1 = r2 - t1;

      /* (xa - xc) + (yb-yd) */
      r2 = r2 + t1;

      /* (ya - yc) + (xb-xd) */
      s1 = s2 + t2;

      /* (ya - yc) - (xb-xd) */
      s2 = s2 - t2;

      /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
      pSrc[2U * i2] = r1 * onebyfftLen;

      /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
      pSrc[(2U * i2) + 1U] = s1 * onebyfftLen;

      /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
      pSrc[2U * i3] = r2 * onebyfftLen;

      /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
      pSrc[(2U * i3) + 1U] = s2 * onebyfftLen;
   }
}


