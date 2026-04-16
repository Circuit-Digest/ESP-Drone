#ifndef QUATCOMPRESS_H
#define QUATCOMPRESS_H

/*

MIT License

Copyright (c) 2016 James A. Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <stdint.h>
#include <math.h>

// assumes input quaternion is normalized. will fail if not.
static inline uint32_t quatcompress(float const q[4])
{
	// we send the values of the quaternion's smallest 3 elements.
	unsigned i_largest = 0;
	for (unsigned i = 1; i < 4; ++i) {
		if (fabsf(q[i]) > fabsf(q[i_largest])) {
			i_largest = i;
		}
	}

	// since -q represents the same rotation as q,
	// transform the quaternion so the largest element is positive.
	// this avoids having to send its sign bit.
	unsigned negate = q[i_largest] < 0;

	// 1/sqrt(2) is the largest possible value 
	// of the second-largest element in a unit quaternion.

	// do compression using sign bit and 9-bit precision per element.
	uint32_t comp = i_largest;
	for (unsigned i = 0; i < 4; ++i) {
		if (i != i_largest) {
			unsigned negbit = (q[i] < 0) ^ negate;
			unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / (float)M_SQRT1_2) + 0.5f;
			comp = (comp << 10) | (negbit << 9) | mag;
		}
	}

	return comp;
}

static inline void quatdecompress(uint32_t comp, float q[4])
{
	unsigned const mask = (1 << 9) - 1;

	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = ((float)M_SQRT1_2) * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}
	q[i_largest] = sqrtf(1.0f - sum_squares);
}

#endif // QUATCOMPRESS_H
