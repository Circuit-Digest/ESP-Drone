#include "xtensa_math.h"
/*
arm_bitreversal_32 PROC
	ADDS     r3,r1,#1
	PUSH     {r4-r6}
	ADDS     r1,r2,#0
	LSRS     r3,r3,#1
arm_bitreversal_32_0 LABEL
	LDRH     r2,[r1,#2]
	LDRH     r6,[r1,#0]
	ADD      r2,r0,r2
	ADD      r6,r0,r6
	LDR      r5,[r2,#0]
	LDR      r4,[r6,#0]
	STR      r5,[r6,#0]
	STR      r4,[r2,#0]
	LDR      r5,[r2,#4]
	LDR      r4,[r6,#4]
	STR      r5,[r6,#4]
	STR      r4,[r2,#4]
	ADDS     r1,r1,#4
	SUBS     r3,r3,#1
	BNE      arm_bitreversal_32_0
	POP      {r4-r6}
	BX       lr
	ENDP
	
*/
void xtensa_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTab)
{
  uint32_t r3 = (bitRevLen + 1) / 2;
  uint32_t *r2, *r6;
  uint32_t r4, r5;	
  while(r3--)
  {
    r2 = (uint32_t *)((uint32_t)pSrc + pBitRevTab[0]);
    r6 = (uint32_t *)((uint32_t)pSrc + pBitRevTab[1]);

    r5 = r2[0];
    r4 = r6[0];
    r6[0] = r5;
    r2[0] = r4;

    r5 = r2[1];
    r4 = r6[1];
    r6[1] = r5;
    r2[1] = r4;

    pBitRevTab += 2;
  }
}