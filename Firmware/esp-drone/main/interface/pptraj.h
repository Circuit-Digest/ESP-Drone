/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

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

/*
Header file for piecewise polynomial trajectories
*/

#pragma once

#include "math3d.h"

#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)


//
// 1d polynomial functions.
// coefficients are stored in ascending order, i.e. p[0] is the constant term.
//

// evaluate a polynomial using horner's rule.
float polyval(float const p[PP_SIZE], float t);

// construct a linear polynomial from p(0) = x0 to p(duration) = x1.
void polylinear(float p[PP_SIZE], float duration, float x0, float x1);

// construct a polynomial from Bezier curve control points
void polybezier(float p[PP_SIZE], float duration, float* x, int dim);

// plan a degree-5 polynomial with the given duration T,
// and given initial/final position, velocity, and acceleration
void poly5(float poly[PP_SIZE], float T,
	float x0, float dx0, float ddx0,
	float xf, float dxf, float ddxf);

// scale a polynomial in place.
void polyscale(float p[PP_SIZE], float s);

// compute the derivate of a polynomial in place.
void polyder(float p[PP_SIZE]);

// e.g. if s==2 the new polynomial will be stretched to take 2x longer
void polystretchtime(float p[PP_SIZE], float s);

// reflect a polynomial about the x-axis, e.g. p_after(-x) == p_before(x)
void polyreflect(float p[PP_SIZE]);


//
// 4d single polynomial piece for x-y-z-yaw, includes duration.
//

struct poly4d
{
	float p[4][PP_SIZE];
	float duration; // TODO use int millis instead?
} __attribute__((packed));

// construct a 4d zero polynomial.
struct poly4d poly4d_zero(float duration);

// construct a 4d linear polynomial.
struct poly4d poly4d_linear(float duration,
	struct vec p0, struct vec p1, float yaw0, float yaw1);

// scale a 4d polynomial in-place.
void poly4d_scale(struct poly4d *p, float x, float y, float z, float yaw);

// shift a 4d polynomial by the given values in-place.
void poly4d_shift(struct poly4d *p, float x, float y, float z, float yaw);
static inline void poly4d_shift_vec(struct poly4d *p, struct vec pos, float yaw) {
	poly4d_shift(p, pos.x, pos.y, pos.z, yaw);
}

// e.g. if s==2 the new polynomial will be stretched to take 2x longer.
void poly4d_stretchtime(struct poly4d *p, float s);

// compute the derivative of a 4d polynomial in-place.
void polyder4d(struct poly4d *p);

// compute loose maximum of acceleration -
// uses L1 norm instead of Euclidean, evaluates polynomial instead of root-finding
float poly4d_max_accel_approx(struct poly4d const *p);


// output of differentially flat 4d polynomials.
struct traj_eval
{
	struct vec pos;
	struct vec vel;
	struct vec acc;
	struct vec omega;
	float yaw;
};

// a traj_eval with all zero members.
struct traj_eval traj_eval_zero(void);

// a special value of traj_eval that indicates an invalid result.
struct traj_eval traj_eval_invalid(void);

// check if a traj_eval represents an invalid result.
bool is_traj_eval_valid(struct traj_eval const *ev);

// evaluate a single polynomial piece
struct traj_eval poly4d_eval(struct poly4d const *p, float t);



// ----------------------------------//
// piecewise polynomial trajectories //
// ----------------------------------//

struct piecewise_traj
{
	float t_begin;
	float timescale;
	struct vec shift;
	unsigned char n_pieces;
	struct poly4d* pieces;
};

static inline float piecewise_duration(struct piecewise_traj const *pp)
{
	float total_dur = 0;
	for (int i = 0; i < pp->n_pieces; ++i) {
		total_dur += pp->pieces[i].duration;
	}
	return total_dur * pp->timescale;
}

void piecewise_plan_5th_order(struct piecewise_traj *p, float duration,
	struct vec p0, float y0, struct vec v0, float dy0, struct vec a0,
	struct vec p1, float y1, struct vec v1, float dy1, struct vec a1);

void piecewise_plan_7th_order_no_jerk(struct piecewise_traj *p, float duration,
	struct vec p0, float y0, struct vec v0, float dy0, struct vec a0,
	struct vec p1, float y1, struct vec v1, float dy1, struct vec a1);

struct traj_eval piecewise_eval(
	struct piecewise_traj const *traj, float t);

struct traj_eval piecewise_eval_reversed(
	struct piecewise_traj const *traj, float t);


static inline bool piecewise_is_finished(struct piecewise_traj const *traj, float t)
{
	return (t - traj->t_begin) >= piecewise_duration(traj);
}
