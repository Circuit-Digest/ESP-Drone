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
Header file for planning state machine
*/

#pragma once

#include "math3d.h"
#include "pptraj.h"
#include "pptraj_compressed.h"

enum trajectory_state
{
	TRAJECTORY_STATE_IDLE            = 0,
	TRAJECTORY_STATE_FLYING          = 1,
	TRAJECTORY_STATE_LANDING         = 3,
};

enum trajectory_type
{
	TRAJECTORY_TYPE_PIECEWISE            = 0,
	TRAJECTORY_TYPE_PIECEWISE_COMPRESSED = 1
};

struct planner
{
	enum trajectory_state state;	// current state
	enum trajectory_type type;      // current type
	bool reversed;					// true, if trajectory should be evaluated in reverse

	union {
		const struct piecewise_traj* trajectory; // pointer to trajectory
		struct piecewise_traj_compressed* compressed_trajectory; // pointer to compressed trajectory
	};

	struct piecewise_traj planned_trajectory; // trajectory for on-board planning
	struct poly4d pieces[1]; // the on-board planner requires a single piece, only
};

// initialize the planner
void plan_init(struct planner *p);

// tell the planner to stop.
// subsequently, plan_is_stopped(p) will return true,
// and it is no longer valid to call plan_current_goal(p).
void plan_stop(struct planner *p);

// query if the planner is stopped.
// currently this is true at startup before we take off,
// and also after an emergency stop.
bool plan_is_stopped(struct planner *p);

// get the planner's current goal.
struct traj_eval plan_current_goal(struct planner *p, float t);

// start a takeoff trajectory.
int plan_takeoff(struct planner *p, struct vec curr_pos, float curr_yaw, float hover_height, float hover_yaw, float duration, float t);

// start a landing trajectory.
int plan_land(struct planner *p, struct vec curr_pos, float curr_yaw, float hover_height, float hover_yaw, float duration, float t);

// move to a given position, then hover there.
int plan_go_to(struct planner *p, bool relative, struct vec hover_pos, float hover_yaw, float duration, float t);

// same as above, but with current state provided from outside.
int plan_go_to_from(struct planner *p, const struct traj_eval *curr_eval, bool relative, struct vec hover_pos, float hover_yaw, float duration, float t);

// start trajectory
int plan_start_trajectory(struct planner *p, const struct piecewise_traj* trajectory, bool reversed);

// start compressed trajectory
int plan_start_compressed_trajectory(struct planner *p, struct piecewise_traj_compressed* trajectory);

// Query if the trjectory is finished
bool plan_is_finished(struct planner *p, float t);
