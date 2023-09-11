#pragma once

/*
The MIT License (MIT)

cmath3d
Copyright (c) 2016-2018 James Alan Preiss

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

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef CMATH3D_ASSERTS
#include <assert.h>
#endif

#ifndef M_PI_F
#define M_PI_F   (3.14159265358979323846f)
#define M_1_PI_F (0.31830988618379067154f)
#define M_PI_2_F (1.57079632679f)
#endif


// ----------------------------- scalars --------------------------------

static inline float fsqr(float x) { return x * x; }
static inline float radians(float degrees) { return (M_PI_F / 180.0f) * degrees; }
static inline float degrees(float radians) { return (180.0f / M_PI_F) * radians; }
static inline float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}
// test two floats for approximate equality using the "consecutive floats have
// consecutive bit representations" property. Argument `ulps` is the number of
// steps to allow. this does not work well for numbers near zero.
// See https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
static inline bool fcloseulps(float a, float b, int ulps) {
	if ((a < 0.0f) != (b < 0.0f)) {
		// Handle negative zero.
		if (a == b) {
			return true;
		}
		return false;
	}
	int ia = *((int *)&a);
	int ib = *((int *)&b);
	return fabsf(ia - ib) <= ulps;
}


// ---------------------------- 3d vectors ------------------------------

struct vec {
	float x; float y; float z;
};

//
// constructors
//

// construct a vector from 3 floats.
static inline struct vec mkvec(float x, float y, float z) {
	struct vec v;
	v.x = x; v.y = y; v.z = z;
	return v;
}
// construct a vector with the same value repeated for x, y, and z.
static inline struct vec vrepeat(float x) {
	return mkvec(x, x, x);
}
// construct a zero-vector.
static inline struct vec vzero(void) {
	return vrepeat(0.0f);
}
// construct the i'th basis vector, i.e. vbasis(0) == (1, 0, 0).
static inline struct vec vbasis(int i) {
	float a[3] = {0.0f, 0.0f, 0.0f};
	a[i] = 1.0f;
	return mkvec(a[0], a[1], a[2]);
}

//
// operators
//

// multiply a vector by a scalar.
static inline struct vec vscl(float s, struct vec v) {
	return mkvec(s * v.x , s * v.y, s * v.z);
}
// negate a vector.
static inline struct vec vneg(struct vec v) {
	return mkvec(-v.x, -v.y, -v.z);
}
// divide a vector by a scalar.
// does not perform divide-by-zero check.
static inline struct vec vdiv(struct vec v, float s) {
	return vscl(1.0f/s, v);
}
// add two vectors.
static inline struct vec vadd(struct vec a, struct vec b) {
	return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}
// subtract a vector from another vector.
static inline struct vec vsub(struct vec a, struct vec b) {
	return vadd(a, vneg(b));
}
// vector dot product.
static inline float vdot(struct vec a, struct vec b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
// element-wise vector multiply.
static inline struct vec veltmul(struct vec a, struct vec b) {
	return mkvec(a.x * b.x, a.y * b.y, a.z * b.z);
}
// element-wise vector divide.
static inline struct vec veltdiv(struct vec a, struct vec b) {
	return mkvec(a.x / b.x, a.y / b.y, a.z / b.z);
}
// element-wise vector reciprocal.
static inline struct vec veltrecip(struct vec a) {
	return mkvec(1.0f / a.x, 1.0f / a.y, 1.0f / a.z);
}
// vector magnitude squared.
static inline float vmag2(struct vec v) {
	return vdot(v, v);
}
// vector magnitude.
static inline float vmag(struct vec v) {
	return sqrtf(vmag2(v));
}
// vector Euclidean distance squared.
static inline float vdist2(struct vec a, struct vec b) {
  return vmag2(vsub(a, b));
}
// vector Euclidean distance.
static inline float vdist(struct vec a, struct vec b) {
  return sqrtf(vdist2(a, b));
}
// normalize a vector (make a unit vector).
static inline struct vec vnormalize(struct vec v) {
	return vdiv(v, vmag(v));
}
// clamp the Euclidean norm of a vector if it exceeds given maximum.
static inline struct vec vclampnorm(struct vec v, float maxnorm) {
	float const norm = vmag(v);
	if (norm > maxnorm) {
		return vscl(maxnorm / norm, v);
	}
	return v;
}
// vector cross product.
static inline struct vec vcross(struct vec a, struct vec b) {
	return mkvec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
// projection of a onto b, where b is a unit vector.
static inline struct vec vprojectunit(struct vec a, struct vec b_unit) {
	return vscl(vdot(a, b_unit), b_unit);
}
// component of a orthogonal to b, where b is a unit vector.
static inline struct vec vorthunit(struct vec a, struct vec b_unit) {
	return vsub(a, vprojectunit(a, b_unit));
}
// element-wise absolute value of vector.
static inline struct vec vabs(struct vec v) {
	return mkvec(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}
// element-wise minimum of vector.
static inline struct vec vmin(struct vec a, struct vec b) {
	return mkvec(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
// element-wise maximum of vector.
static inline struct vec vmax(struct vec a, struct vec b) {
	return mkvec(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}
// element-wise clamp of vector.
static inline struct vec vclamp(struct vec v, struct vec lower, struct vec upper) {
	return vmin(upper, vmax(v, lower));
}
// element-wise clamp of vector to range centered on zero.
static inline struct vec vclampabs(struct vec v, struct vec abs_upper) {
	return vclamp(v, vneg(abs_upper), abs_upper);
}
// largest scalar element of vector.
static inline float vmaxelt(struct vec v) {
	return fmax(fmax(v.x, v.y), v.z);
}
// least (most negative) scalar element of vector.
static inline float vminelt(struct vec v) {
	return fmin(fmin(v.x, v.y), v.z);
}
// L1 norm (aka Minkowski, Taxicab, Manhattan norm) of a vector.
static inline float vnorm1(struct vec v) {
	return fabsf(v.x) + fabsf(v.y) + fabsf(v.z);
}

//
// comparisons, including partial orderings
//

// compare two vectors for exact equality.
static inline bool veq(struct vec a, struct vec b) {
	return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}
// compare two vectors for exact inequality.
static inline bool vneq(struct vec a, struct vec b) {
	return !veq(a, b);
}
// compare two vectors for near-equality with user-defined threshold.
static inline bool veqepsilon(struct vec a, struct vec b, float epsilon) {
	struct vec diffs = vabs(vsub(a, b));
	return diffs.x < epsilon && diffs.y < epsilon && diffs.z < epsilon;
}
// all(element-wise less-than)
static inline bool vless(struct vec a, struct vec b) {
	return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}
// all(element-wise less-than-or-equal)
static inline bool vleq(struct vec a, struct vec b) {
	return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
}
// all(element-wise greater-than)
static inline bool vgreater(struct vec a, struct vec b) {
	return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}
// all(element-wise greater-than-or-equal)
static inline bool vgeq(struct vec a, struct vec b) {
	return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
}
// test if any element of a vector is NaN.
static inline bool visnan(struct vec v) {
	return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

//
// special functions to ease the pain of writing vector math in C.
//

// add 3 vectors.
static inline struct vec vadd3(struct vec a, struct vec b, struct vec c) {
	return vadd(vadd(a, b), c);
}
// add 4 vectors.
static inline struct vec vadd4(struct vec a, struct vec b, struct vec c, struct vec d) {
	// TODO: make sure it compiles to optimal code
	return vadd(vadd(a, b), vadd(c, d));
}
// subtract b and c from a.
static inline struct vec vsub2(struct vec a, struct vec b, struct vec c) {
	return vadd3(a, vneg(b), vneg(c));
}

//
// conversion to/from raw float and double arrays; array-like access.
//

// load a vector from a double array.
static inline struct vec vload(double const *d) {
	return mkvec(d[0], d[1], d[2]);
}
// store a vector into a double array.
static inline void vstore(struct vec v, double *d) {
	d[0] = (double)v.x; d[1] = (double)v.y; d[2] = (double)v.z;
}
// load a vector from a float array.
static inline struct vec vloadf(float const *f) {
	return mkvec(f[0], f[1], f[2]);
}
// store a vector into a float array.
static inline void vstoref(struct vec v, float *f) {
	f[0] = v.x; f[1] = v.y; f[2] = v.z;
}
// index a vector like a 3-element array.
static inline float vindex(struct vec v, int i) {
	return ((float const *)&v.x)[i];
}


// ---------------------------- 3x3 matrices ------------------------------

struct mat33 {
	float m[3][3];
};

//
// constructors
//

// construct a zero matrix.
static inline struct mat33 mzero(void) {
	struct mat33 m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m.m[i][j] = 0;
		}
	}
	return m;
}
// construct a matrix with the given diagonal.
static inline struct mat33 mdiag(float a, float b, float c) {
	struct mat33 m = mzero();
	m.m[0][0] = a;
	m.m[1][1] = b;
	m.m[2][2] = c;
	return m;
}
// construct the matrix a * I for scalar a.
static inline struct mat33 meyescl(float a) {
	return mdiag(a, a, a);
}
// construct an identity matrix.
static inline struct mat33 meye(void) {
	return meyescl(1.0f);
}
// construct a matrix from three column vectors.
static inline struct mat33 mcolumns(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	m.m[0][0] = a.x;
	m.m[1][0] = a.y;
	m.m[2][0] = a.z;

	m.m[0][1] = b.x;
	m.m[1][1] = b.y;
	m.m[2][1] = b.z;

	m.m[0][2] = c.x;
	m.m[1][2] = c.y;
	m.m[2][2] = c.z;

	return m;
}
// construct a matrix from three row vectors.
static inline struct mat33 mrows(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	vstoref(a, m.m[0]);
	vstoref(b, m.m[1]);
	vstoref(c, m.m[2]);
	return m;
}
// construct the matrix A from vector v such that Ax = cross(v, x)
static inline struct mat33 mcrossmat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}

//
// accessors
//

// return one column of a matrix as a vector.
static inline struct vec mcolumn(struct mat33 m, int col) {
	return mkvec(m.m[0][col], m.m[1][col], m.m[2][col]);
}
// return one row of a matrix as a vector.
static inline struct vec mrow(struct mat33 m, int row) {
	return vloadf(m.m[row]);
}

//
// operators
//

// matrix transpose.
static inline struct mat33 mtranspose(struct mat33 m) {
	struct mat33 mt;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			mt.m[i][j] = m.m[j][i];
		}
	}
	return mt;
}
// multiply a matrix by a scalar.
static inline struct mat33 mscl(float s, struct mat33 a) {
	struct mat33 sa;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sa.m[i][j] = s * a.m[i][j];
		}
	}
	return sa;
}
// negate a matrix.
static inline struct mat33 mneg(struct mat33 a) {
	return mscl(-1.0, a);
}
// add two matrices.
static inline struct mat33 madd(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] + b.m[i][j];
		}
	}
	return c;
}
// subtract two matrices.
static inline struct mat33 msub(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] - b.m[i][j];
		}
	}
	return c;
}
// multiply a matrix by a vector.
static inline struct vec mvmul(struct mat33 a, struct vec v) {
	float x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z;
	float y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z;
	float z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z;
	return mkvec(x, y, z);
}
// multiply two matrices.
static inline struct mat33 mmul(struct mat33 a, struct mat33 b) {
	struct mat33 ab;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float accum = 0;
			for (int k = 0; k < 3; ++k) {
				accum += a.m[i][k] * b.m[k][j];
			}
			ab.m[i][j] = accum;
		}
	}
	return ab;
}
// add a scalar along the diagonal, i.e. a + dI.
static inline struct mat33 maddridge(struct mat33 a, float d) {
	a.m[0][0] += d;
	a.m[1][1] += d;
	a.m[2][2] += d;
	return a;
}
// test if any element of a matrix is NaN.
static inline bool misnan(struct mat33 m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (isnan(m.m[i][j])) {
				return true;
			}
		}
	}
	return false;
}

// set a 3x3 block within a big row-major matrix.
// block: pointer to the upper-left element of the block in the big matrix.
// stride: the number of columns in the big matrix.
static inline void set_block33_rowmaj(float *block, int stride, struct mat33 const *m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			block[j] = m->m[i][j];
		}
		block += stride;
	}
}

//
// special functions to ease the pain of writing vector math in C.
//

// add three matrices.
static inline struct mat33 madd3(struct mat33 a, struct mat33 b, struct mat33 c) {
	return madd(madd(a, b), c);
}

//
// 3D rotation constructors & operators
//

// construct equivalent rotation matrix for axis-angle rotation.
// assumes input axis is normalized, angle in radians.
static inline struct mat33 maxisangle(struct vec axis, float angle) {
	// Rodrigues formula
	struct mat33 const K = mcrossmat(axis);
	return madd3(
		meye(),
		mscl(sinf(angle), K),
		mscl(1.0f - cosf(angle), mmul(K, K))
	);
}
// rotation about x axis by given angle (radians)
static inline struct mat33 mrotx(float angle) {
	return maxisangle(mkvec(1.0f, 0.0f, 0.0f), angle);
}
// rotation about y axis by given angle (radians)
static inline struct mat33 mroty(float angle) {
	return maxisangle(mkvec(0.0f, 1.0f, 0.0f), angle);
}
// rotation about z axis by given angle (radians)
static inline struct mat33 mrotz(float angle) {
	return maxisangle(mkvec(0.0f, 0.0f, 1.0f), angle);
}
// TODO: these might be faster if written by hand,
// but these are correct and the trig is probably the slow part anyway


// Matrix TODO: inv, solve, eig, 9 floats ctor, axis-aligned rotations


// ---------------------------- quaternions ------------------------------

struct quat {
	float x;
	float y;
	float z;
	float w;
};

//
// constructors
//

// construct a quaternion from its x, y, z, w elements.
static inline struct quat mkquat(float x, float y, float z, float w) {
	struct quat q;
	q.x = x; q.y = y; q.z = z; q.w = w;
	return q;
}
// construct a quaternion from a vector containing (x, y, z) and a scalar w.
// note that this is NOT an axis-angle constructor.
static inline struct quat quatvw(struct vec v, float w) {
	struct quat q;
	q.x = v.x; q.y = v.y; q.z = v.z;
	q.w = w;
	return q;
}
// construct an identity quaternion.
static inline struct quat qeye(void) {
	return mkquat(0, 0, 0, 1);
}
// construct a quaternion from an axis and angle of rotation.
// does not assume axis is normalized.
static inline struct quat qaxisangle(struct vec axis, float angle) {
	float scale = sinf(angle / 2) / vmag(axis);
	struct quat q;
	q.x = scale * axis.x;
	q.y = scale * axis.y;
	q.z = scale * axis.z;
	q.w = cosf(angle/2);
	return q;
}

// fwd declare, needed in some ctors
static inline struct quat qnormalize(struct quat q);

// construct a quaternion such that q * a = b,
// and the rotation axis is orthogonal to the plane defined by a and b,
// and the rotation is less than 180 degrees.
// assumes a and b are unit vectors.
// does not handle degenerate case where a = -b. returns all-zero quat
static inline struct quat qvectovec(struct vec a, struct vec b) {
	struct vec const cross = vcross(a, b);
	float const sinangle = vmag(cross);
	float const cosangle = vdot(a, b);
	// avoid taking sqrt of negative number due to floating point error.
	// TODO: find tighter exact bound
	float const EPS_ANGLE = 1e-6;
	if (sinangle < EPS_ANGLE) {
		if (cosangle > 0.0f) return qeye();
		else return mkquat(0.0f, 0.0f, 0.0f, 0.0f); // degenerate
	}
	float const halfcos = 0.5f * cosangle;
	// since angle is < 180deg, the positive sqrt is always correct
	float const sinhalfangle = sqrtf(fmax(0.5f - halfcos, 0.0f));
	float const coshalfangle = sqrtf(fmax(0.5f + halfcos, 0.0f));
	struct vec const qimag = vscl(sinhalfangle / sinangle, cross);
	float const qreal = coshalfangle;
	return quatvw(qimag, qreal);
}
// construct from (roll, pitch, yaw) Euler angles using Tait-Bryan convention
// (yaw, then pitch about new pitch axis, then roll about new roll axis)
static inline struct quat rpy2quat(struct vec rpy) {
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	float r = rpy.x;
	float p = rpy.y;
	float y = rpy.z;
	float cr = cosf(r / 2.0f); float sr = sinf(r / 2.0f);
	float cp = cosf(p / 2.0f); float sp = sinf(p / 2.0f);
	float cy = cosf(y / 2.0f); float sy = sinf(y / 2.0f);

	float qx = sr * cp * cy -  cr * sp * sy;
	float qy = cr * sp * cy +  sr * cp * sy;
	float qz = cr * cp * sy -  sr * sp * cy;
	float qw = cr * cp * cy +  sr * sp * sy;

	return mkquat(qx, qy, qz, qw);
}
// APPROXIMATE construction of a quaternion from small (roll, pitch, yaw) Euler angles
// without computing any trig functions. only produces useful result for small angles.
// Example application is integrating a gyroscope when the angular velocity
// of the object is small compared to the sampling frequency.
static inline struct quat rpy2quat_small(struct vec rpy) {
	// TODO: cite source, but can be derived from rpy2quat under first-order approximation:
	// sin(epsilon) = epsilon, cos(epsilon) = 1, epsilon^2 = 0
	float q2 = vmag2(rpy) / 4.0f;
	if (q2 < 1) {
		return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
	}
	else {
		float w = 1.0f / sqrtf(1.0f + q2);
		return quatvw(vscl(w/2, rpy), w);
	}
}
// construct quaternion from orthonormal matrix.
static inline struct quat mat2quat(struct mat33 m) {
	float w = sqrtf(fmax(0.0f, 1.0f + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2.0f;
	float x = sqrtf(fmax(0.0f, 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2.0f;
	float y = sqrtf(fmax(0.0f, 1.0f - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2.0f;
	float z = sqrtf(fmax(0.0f, 1.0f - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2.0f;
	x = copysign(x, m.m[2][1] - m.m[1][2]);
	y = copysign(y, m.m[0][2] - m.m[2][0]);
	z = copysign(z, m.m[1][0] - m.m[0][1]);
	return mkquat(x, y, z, w);
}

//
// conversions to other parameterizations of 3D rotations
//

// convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
// (yaw, then pitch about new pitch axis, then roll about new roll axis)
static inline struct vec quat2rpy(struct quat q) {
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	struct vec v;
	v.x = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1 - 2 * (fsqr(q.x) + fsqr(q.y))); // roll
	v.y = asinf(2.0f * (q.w * q.y - q.x * q.z)); // pitch
	v.z = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1 - 2 * (fsqr(q.y) + fsqr(q.z))); // yaw
	return v;
}
// compute the axis of a quaternion's axis-angle decomposition.
static inline struct vec quat2axis(struct quat q) {
	// TODO this is not numerically stable for tiny rotations
	float s = 1.0f / sqrtf(1.0f - q.w * q.w);
	return vscl(s, mkvec(q.x, q.y, q.z));
}
// compute the angle of a quaternion's axis-angle decomposition.
// result lies in domain (-pi, pi].
static inline float quat2angle(struct quat q) {
	float angle = 2 * acosf(q.w);
	if (angle > M_PI_F) {
		angle -= 2.0f * M_PI_F;
	}
	return angle;
}
// vector containing the imaginary part of the quaternion, i.e. (x, y, z)
static inline struct vec quatimagpart(struct quat q) {
	return mkvec(q.x, q.y, q.z);
}
// convert a quaternion into a 3x3 rotation matrix.
static inline struct mat33 quat2rotmat(struct quat q) {
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;

	struct mat33 m;
	m.m[0][0] = 1 - 2*y*y - 2*z*z;
	m.m[0][1] = 2*x*y - 2*z*w;
	m.m[0][2] = 2*x*z + 2*y*w,
	m.m[1][0] = 2*x*y + 2*z*w;
	m.m[1][1] = 1 - 2*x*x - 2*z*z;
	m.m[1][2] = 2*y*z - 2*x*w,
	m.m[2][0] = 2*x*z - 2*y*w;
	m.m[2][1] = 2*y*z + 2*x*w;
	m.m[2][2] = 1 - 2*x*x - 2*y*y;
	return m;
}

//
// operators
//

// rotate a vector by a quaternion.
static inline struct vec qvrot(struct quat q, struct vec v) {
	// from http://gamedev.stackexchange.com/a/50545 - TODO find real citation
	struct vec qv = mkvec(q.x, q.y, q.z);
	return vadd3(
		vscl(2.0f * vdot(qv, v), qv),
		vscl(q.w * q.w - vmag2(qv), v),
		vscl(2.0f * q.w, vcross(qv, v))
	);
}
// multiply (compose) two quaternions
// such that qvrot(qqmul(q, p), v) == qvrot(q, qvrot(p, v)).
static inline struct quat qqmul(struct quat q, struct quat p) {
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
	return mkquat(x, y, z, w);
}
// invert a quaternion.
static inline struct quat qinv(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, q.w);
}
// negate a quaternion.
// this represents the same rotation, but is still sometimes useful.
static inline struct quat qneg(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, -q.w);
}
// return a quaternion representing the same rotation
// but with a positive real term (q.w).
// useful to collapse the double-covering of SO(3) by the quaternions.
static inline struct quat qposreal(struct quat q) {
	if (q.w < 0) return qneg(q);
	return q;
}
// quaternion dot product. is cosine of angle between them.
static inline float qdot(struct quat a, struct quat b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
static inline float qanglebetween(struct quat a, struct quat b) {
	float const dot = qdot(qposreal(a), qposreal(b));
	// prevent acos domain issues
	if (dot > 1.0f - 1e9f) return 0.0f;
	if (dot < -1.0f + 1e9f) return M_PI_F;
	return acosf(dot);
}
static inline bool qeq(struct quat a, struct quat b) {
	return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
// normalize a quaternion.
// typically used to mitigate precision errors.
static inline struct quat qnormalize(struct quat q) {
	float s = 1.0f / sqrtf(qdot(q, q));
	return mkquat(s*q.x, s*q.y, s*q.z, s*q.w);
}
// update an attitude estimate quaternion with a reading from a gyroscope
// over the timespan dt. Gyroscope is assumed (roll, pitch, yaw)
// angular velocities in radians per second.
static inline struct quat quat_gyro_update(struct quat quat, struct vec gyro, float const dt) {
	// from "Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005
	struct quat q1;
	float const r = (dt / 2) * gyro.x;
	float const p = (dt / 2) * gyro.y;
	float const y = (dt / 2) * gyro.z;

	q1.x =    quat.x + y*quat.y - p*quat.z + r*quat.w;
	q1.y = -y*quat.x +   quat.y + r*quat.z + p*quat.w;
	q1.z =  p*quat.x - r*quat.y +   quat.z + y*quat.w;
	q1.w = -r*quat.x - p*quat.y - y*quat.z +   quat.w;
	return q1;
}
// normalized linear interpolation. s should be between 0 and 1.
static inline struct quat qnlerp(struct quat a, struct quat b, float t) {
	float s = 1.0f - t;
	return qnormalize(mkquat(
		s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w));
}
// spherical linear interpolation. s should be between 0 and 1.
static inline struct quat qslerp(struct quat a, struct quat b, float t)
{
	// from "Animating Rotation with Quaternion Curves", Ken Shoemake, 1985
	float dp = qdot(a, b);
	if (dp < 0) {
		dp = -dp;
		b = qneg(b);
	}

	if (dp > 0.99f) {
		// fall back to linear interpolation to avoid div-by-zero
		return qnlerp(a, b, t);
	}
	else {
		float theta = acosf(dp);
		float s = sinf(theta * (1 - t)) / sinf(theta);
		t = sinf(theta * t) / sinf(theta);
		return mkquat(
			s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w);
	}
}

//
// conversion to/from raw float and double arrays.
//

// load a quaternion from a raw double array.
static inline struct quat qload(double const *d) {
	return mkquat(d[0], d[1], d[2], d[3]);
}
// store a quaternion into a raw double array.
static inline void qstore(struct quat q, double *d) {
	d[0] = (double)q.x; d[1] = (double)q.y; d[2] = (double)q.z; d[3] = (double)q.w;
}
// load a quaternion from a raw float array.
static inline struct quat qloadf(float const *f) {
	return mkquat(f[0], f[1], f[2], f[3]);
}
// store a quaternion into a raw float array.
static inline void qstoref(struct quat q, float *f) {
	f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}


// ------------------------ convex sets in R^3 ---------------------------

// project v onto the halfspace H = {x : a^T x <= b}, where a is a unit vector.
// If v lies in H, returns v. Otherwise, returns the closest point, which
// minimizes |x - v|_2, and will satisfy a^T x = b.
static inline struct vec vprojecthalfspace(struct vec x, struct vec a_unit, float b) {
	float ax = vdot(a_unit, x);
	if (ax <= b) {
		return x;
	}
	return vadd(x, vscl(b - ax, a_unit));
}

// test if v lies in the convex polytope defined by linear inequalities Ax <= b.
// A: n x 3 matrix, row-major. Each row must have L2 norm of 1.
// b: n vector.
// tolerance: Allow violations up to Ax <= b + tolerance.
static inline bool vinpolytope(struct vec v, float const A[], float const b[], int n, float tolerance)
{
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		if (vdot(a, v) > b[i] + tolerance) {
			return false;
		}
	}
	return true;
}

// Finds the intersection between a ray and a convex polytope boundary.
// The polytope is defined by the linear inequalities Ax <= b.
// The ray must originate within the polytope.
//
// Args:
//   origin: Origin of the ray. Must lie within the polytope.
//   direction: Direction of the ray.
//   A: n x 3 matrix, row-major. Each row must have L2 norm of 1.
//   b: n vector.
//   n: Number of inequalities (rows in A).
//
// Returns:
//   s: positive float such that (origin + s * direction) is on the polytope
//     boundary. If the ray does not intersect the polytope -- for example, if
//     the polytope is unbounded -- then float INFINITY will be returned.
//     If the polytope is empty, then a negative number will be returned.
//     If `origin` does not lie within the polytope, return value is undefined.
//   active_row: output argument. The row in A (face of the polytope) that
//     the ray intersects. The point (origin + s * direction) will satisfy the
//     equation in that row with equality. If the ray intersects the polytope
//     at an intersection of two or more faces, active_row will be an arbitrary
//     member of the intersecting set. Optional, can be NULL.
//
static inline float rayintersectpolytope(struct vec origin, struct vec direction, float const A[], float const b[], int n, int *active_row)
{
	#ifdef CMATH3D_ASSERTS
	// check for normalized input.
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		assert(fabsf(vmag2(a) - 1.0f) < 1e-6f);
	}
	#endif

	float min_s = INFINITY;
	int min_row = -1;

	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		float a_dir = vdot(a, direction);
		if (a_dir <= 0.0f) {
			// The ray points away from or parallel to the polytope face,
			// so it will never intersect.
			continue;
		}
		// Solve for the intersection point of this halfspace algebraically.
		float s = (b[i] - vdot(a, origin)) / a_dir;
		if (s < min_s) {
			min_s = s;
			min_row = i;
		}
	}

	if (active_row != NULL) {
		*active_row = min_row;
	}
	return min_s;
}

// Projects v onto the convex polytope defined by linear inequalities Ax <= b.
// Returns argmin_{x: Ax <= b} |x - v|_2. Uses Dykstra's (not Dijkstra's!)
// projection algorithm [1] with robust stopping criteria [2].
//
// Args:
//   v: vector to project into the polytope.
//   A: n x 3 matrix, row-major. Each row must have L2 norm of 1.
//   b: n vector.
//   work: n x 3 matrix. will be overwritten. input values are not used.
//   tolerance: Stop when *approximately* violates the polytope constraints
//     by no more than this value. Not exact - be conservative if needed.
//   maxiters: Terminate after this many iterations regardless of convergence.
//
// Returns:
//   The projection of v into the polytope.
//
// References:
//   [1] Boyle, J. P., and Dykstra, R. L. (1986). A Method for Finding
//       Projections onto the Intersection of Convex Sets in Hilbert Spaces.
//       Lecture Notes in Statistics, 28–47. doi:10.1007/978-1-4613-9940-7_3
//   [2] Birgin, E. G., and Raydan, M. (2005). Robust Stopping Criteria for
//       Dykstra's Algorithm. SIAM J. Scientific Computing 26(4): 1405-1414.
//       doi:10.1137/03060062X
//
static inline struct vec vprojectpolytope(struct vec v, float const A[], float const b[], float work[], int n, float tolerance, int maxiters)
{
	// early bailout.
	if (vinpolytope(v, A, b, n, tolerance)) {
		return v;
	}

	#ifdef CMATH3D_ASSERTS
	// check for normalized input.
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		assert(fabsf(vmag2(a) - 1.0f) < 1e-6f);
	}
	#endif

	float *z = work;
	for (int i = 0; i < 3 * n; ++i) {
		z[i] = 0.0f;
	}

	// For user-friendliness, we accept a tolerance value in terms of
	// the Euclidean magnitude of the polytope violation. However, the
	// stopping criteria we wish to use is the robust one of [2] -
	// specifically, the expression called c_I^k - which is based on the
	// sum of squared projection residuals. This is a feeble attempt to get
	// a ballpark tolerance value that is roughly equivalent.
	float const tolerance2 = n * fsqr(tolerance) / 10.0f;
	struct vec x = v;

	for (int iter = 0; iter < maxiters; ++iter) {
		float c = 0.0f;
		for (int i = 0; i < n; ++i) {
			struct vec x_old = x;
			struct vec ai = vloadf(A + 3 * i);
			struct vec zi_old = vloadf(z + 3 * i);
			x = vprojecthalfspace(vsub(x_old, zi_old), ai, b[i]);
			struct vec zi = vadd3(x, vneg(x_old), zi_old);
			vstoref(zi, z + 3 * i);
			c += vdist2(zi_old, zi);
		}
		if (c < tolerance2) {
			return x;
		}
	}
	return x;
}


// Overall TODO: lines? segments? planes? axis-aligned boxes? spheres?
