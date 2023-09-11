/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller.
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "controller_indi.h"
#include "math3d.h"

static float thrust_threshold = 300.0f;
static float bound_control_input = 32000.0f;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;
static float roll_kp = 5.0f;
static float pitch_kp = 5.0f;
static float yaw_kp = 5.0f;

static float attYawError; 

static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static vector_t refOuterINDI;				// Reference values from outer loop INDI
static bool outerLoopActive = false ; 		// if 1, outer loop INDI is activated

static struct IndiVariables indi = {
		.g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R},
		.g2 = STABILIZATION_INDI_G2_R,
		.reference_acceleration = {
				STABILIZATION_INDI_REF_ERR_P,
				STABILIZATION_INDI_REF_ERR_Q,
				STABILIZATION_INDI_REF_ERR_R,
				STABILIZATION_INDI_REF_RATE_P,
				STABILIZATION_INDI_REF_RATE_Q,
				STABILIZATION_INDI_REF_RATE_R
		},
		.act_dyn = {STABILIZATION_INDI_ACT_DYN_P, STABILIZATION_INDI_ACT_DYN_Q, STABILIZATION_INDI_ACT_DYN_R},
		.filt_cutoff = STABILIZATION_INDI_FILT_CUTOFF,
		.filt_cutoff_r = STABILIZATION_INDI_FILT_CUTOFF_R,
};

static inline void float_rates_zero(struct FloatRates *fr) {
	fr->p = 0.0f;
	fr->q = 0.0f;
	fr->r = 0.0f;
}

void indi_init_filters(void)
{
	// tau = 1/(2*pi*Fc)
	float tau = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff);
	float tau_r = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff_r);
	float tau_axis[3] = {tau, tau, tau_r};
	float sample_time = 1.0f / ATTITUDE_RATE;
	// Filtering of gyroscope and actuators
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0f);
		init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0f);
	}
}

/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
static inline void filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
{
	update_butterworth_2_low_pass(&filter[0], new_values->p);
	update_butterworth_2_low_pass(&filter[1], new_values->q);
	update_butterworth_2_low_pass(&filter[2], new_values->r);
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
static inline void finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
{
	for (int8_t i = 0; i < 3; i++) {
		output[i] = (filter[i].o[0] - filter[i].o[1]) * ATTITUDE_RATE;
	}
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}


void controllerINDIInit(void)
{
	/*
	 * TODO
	 * Can this also be called during flight, for instance when switching controllers?
	 * Then the filters should not be reset to zero but to the current values of sensors and actuators.
	 */
	float_rates_zero(&indi.angular_accel_ref);
	float_rates_zero(&indi.u_act_dyn);
	float_rates_zero(&indi.u_in);

	// Re-initialize filters
	indi_init_filters();

	attitudeControllerInit(ATTITUDE_UPDATE_DT);
	positionControllerInit();
	positionControllerINDIInit();
}

bool controllerINDITest(void)
{
	bool pass = true;

	pass &= attitudeControllerTest();

	return pass;
}

void controllerINDI(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors,
		const state_t *state,
		const uint32_t tick)
{

	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		// Rate-controled YAW is moving YAW angle setpoint
		if (setpoint->mode.yaw == modeVelocity) {
			attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
			while (attitudeDesired.yaw > 180.0f)
				attitudeDesired.yaw -= 360.0f;
			while (attitudeDesired.yaw < -180.0f)
				attitudeDesired.yaw += 360.0f;
		} else {
			attitudeDesired.yaw = setpoint->attitude.yaw;
		}
	}

	if (RATE_DO_EXECUTE(POSITION_RATE, tick) && !outerLoopActive) {
		positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
	}

	/*
	 * Skipping calls faster than ATTITUDE_RATE
	 */
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

		// Call outer loop INDI (position controller)
		if (outerLoopActive) {
			positionControllerINDI(sensors, setpoint, state, &refOuterINDI);
		}

		// Switch between manual and automatic position control
		if (setpoint->mode.z == modeDisable) {
				// INDI position controller not active, INDI attitude controller is main loop
				actuatorThrust = setpoint->thrust;
		} else{
			if (outerLoopActive) {
				// INDI position controller active, INDI attitude controller becomes inner loop
				actuatorThrust = refOuterINDI.z;
			}
		}
		if (setpoint->mode.x == modeDisable) {

				// INDI position controller not active, INDI attitude controller is main loop
				attitudeDesired.roll = setpoint->attitude.roll;
			
		}else{
			if (outerLoopActive) {
				// INDI position controller active, INDI attitude controller becomes inner loop
				attitudeDesired.roll = refOuterINDI.x;
			}
		}

		if (setpoint->mode.y == modeDisable) {

				// INDI position controller not active, INDI attitude controller is main loop
				attitudeDesired.pitch = setpoint->attitude.pitch;
			
		}else{
			if (outerLoopActive) {
				// INDI position controller active, INDI attitude controller becomes inner loop
				attitudeDesired.pitch = refOuterINDI.y;
			}
		}
		

//	    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
//	                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
//	                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

		rateDesired.roll = roll_kp*(attitudeDesired.roll - state->attitude.roll);
		rateDesired.pitch = pitch_kp*(attitudeDesired.pitch - state->attitude.pitch);
		//rateDesired.yaw = yaw_kp*(attitudeDesired.yaw - state->attitude.yaw);
		attYawError = attitudeDesired.yaw - state->attitude.yaw;		
		attYawError = capAngle(attYawError);
		rateDesired.yaw = yaw_kp*attYawError;


		// For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
		// value. Also reset the PID to avoid error buildup, which can lead to unstable
		// behavior if level mode is engaged later
		if (setpoint->mode.roll == modeVelocity) {
			rateDesired.roll = setpoint->attitudeRate.roll;
			attitudeControllerResetRollAttitudePID();
		}
		if (setpoint->mode.pitch == modeVelocity) {
			rateDesired.pitch = setpoint->attitudeRate.pitch;
			attitudeControllerResetPitchAttitudePID();
		}

		/*
		 * 1 - Update the gyro filter with the new measurements.
		 */

		float stateAttitudeRateRoll = radians(sensors->gyro.x);
		float stateAttitudeRatePitch = -radians(sensors->gyro.y); // Account for Crazyflie coordinate system
		float stateAttitudeRateYaw = radians(sensors->gyro.z);

		struct FloatRates body_rates = {
				.p = stateAttitudeRateRoll,
				.q = stateAttitudeRatePitch,
				.r = stateAttitudeRateYaw,
		};
		filter_pqr(indi.rate, &body_rates);


		/*
		 * 2 - Calculate the derivative with finite difference.
		 */

		finite_difference_from_filter(indi.rate_d, indi.rate);


		/*
		 * 3 - same filter on the actuators (or control_t values), using the commands from the previous timestep.
		 */
		filter_pqr(indi.u, &indi.u_act_dyn);


		/*
		 * 4 - Calculate the desired angular acceleration by:
		 * 4.1 - Rate_reference = P * attitude_error, where attitude error can be calculated with your favorite
		 * algorithm. You may even use a function that is already there, such as attitudeControllerCorrectAttitudePID(),
		 * though this will be inaccurate for large attitude errors, but it will be ok for now.
		 * 4.2 Angular_acceleration_reference = D * (rate_reference – rate_measurement)
		 */

		float attitude_error_p = radians(rateDesired.roll) - stateAttitudeRateRoll;
		float attitude_error_q = radians(rateDesired.pitch) - stateAttitudeRatePitch;
		float attitude_error_r = radians(rateDesired.yaw) - stateAttitudeRateYaw;

		indi.angular_accel_ref.p = indi.reference_acceleration.err_p * attitude_error_p;

		indi.angular_accel_ref.q = indi.reference_acceleration.err_q * attitude_error_q;

		indi.angular_accel_ref.r = indi.reference_acceleration.err_r * attitude_error_r;

		/*
		 * 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
		 */

		//Increment in angular acceleration requires increment in control input
		//G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
		//It takes care of the angular acceleration caused by the change in rotation rate of the propellers
		//(they have significant inertia, see the paper mentioned in the header for more explanation)
		indi.du.p = 1.0f / indi.g1.p * (indi.angular_accel_ref.p - indi.rate_d[0]);
		indi.du.q = 1.0f / indi.g1.q * (indi.angular_accel_ref.q - indi.rate_d[1]);
		indi.du.r = 1.0f / (indi.g1.r - indi.g2) * (indi.angular_accel_ref.r - indi.rate_d[2] - indi.g2 * indi.du.r);


		/*
		 * 6. Add delta_commands to commands and bound to allowable values
		 */

		indi.u_in.p = indi.u[0].o[0] + indi.du.p;
		indi.u_in.q = indi.u[1].o[0] + indi.du.q;
		indi.u_in.r = indi.u[2].o[0] + indi.du.r;

		//bound the total control input
		indi.u_in.p = clamp(indi.u_in.p, -1.0f*bound_control_input, bound_control_input);
		indi.u_in.q = clamp(indi.u_in.q, -1.0f*bound_control_input, bound_control_input);
		indi.u_in.r = clamp(indi.u_in.r, -1.0f*bound_control_input, bound_control_input);

		//Propagate input filters
		//first order actuator dynamics
		indi.u_act_dyn.p = indi.u_act_dyn.p + indi.act_dyn.p * (indi.u_in.p - indi.u_act_dyn.p);
		indi.u_act_dyn.q = indi.u_act_dyn.q + indi.act_dyn.q * (indi.u_in.q - indi.u_act_dyn.q);
		indi.u_act_dyn.r = indi.u_act_dyn.r + indi.act_dyn.r * (indi.u_in.r - indi.u_act_dyn.r);

	}

	indi.thrust = actuatorThrust;
	r_roll = radians(sensors->gyro.x);
	r_pitch = -radians(sensors->gyro.y);
	r_yaw = radians(sensors->gyro.z);
	accelz = sensors->acc.z;

	//Don't increment if thrust is off
	//TODO: this should be something more elegant, but without this the inputs
	//will increment to the maximum before even getting in the air.
	if(indi.thrust < thrust_threshold) {
		float_rates_zero(&indi.angular_accel_ref);
		float_rates_zero(&indi.u_act_dyn);
		float_rates_zero(&indi.u_in);

		if(indi.thrust == 0){
			attitudeControllerResetAllPID();
			positionControllerResetAllPID();

			// Reset the calculated YAW angle for rate control
			attitudeDesired.yaw = state->attitude.yaw;
		}
	}

	/*  INDI feedback */
	control->thrust = indi.thrust;
	control->roll = indi.u_in.p;
	control->pitch = indi.u_in.q;
	control->yaw  = indi.u_in.r;

}

PARAM_GROUP_START(ctrlINDI)
PARAM_ADD(PARAM_FLOAT, thrust_threshold, &thrust_threshold)
PARAM_ADD(PARAM_FLOAT, bound_ctrl_input, &bound_control_input)
PARAM_ADD(PARAM_FLOAT, roll_kp, &roll_kp)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pitch_kp)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &yaw_kp)
PARAM_ADD(PARAM_FLOAT, g1_p, &indi.g1.p)
PARAM_ADD(PARAM_FLOAT, g1_q, &indi.g1.q)
PARAM_ADD(PARAM_FLOAT, g1_r, &indi.g1.r)
PARAM_ADD(PARAM_FLOAT, g2, &indi.g2)
PARAM_ADD(PARAM_FLOAT, ref_err_p, &indi.reference_acceleration.err_p)
PARAM_ADD(PARAM_FLOAT, ref_err_q, &indi.reference_acceleration.err_q)
PARAM_ADD(PARAM_FLOAT, ref_err_r, &indi.reference_acceleration.err_r)
PARAM_ADD(PARAM_FLOAT, ref_rate_p, &indi.reference_acceleration.rate_p)
PARAM_ADD(PARAM_FLOAT, ref_rate_q, &indi.reference_acceleration.rate_q)
PARAM_ADD(PARAM_FLOAT, ref_rate_r, &indi.reference_acceleration.rate_r)
PARAM_ADD(PARAM_FLOAT, act_dyn_p, &indi.act_dyn.p)
PARAM_ADD(PARAM_FLOAT, act_dyn_q, &indi.act_dyn.q)
PARAM_ADD(PARAM_FLOAT, act_dyn_r, &indi.act_dyn.r)
PARAM_ADD(PARAM_FLOAT, filt_cutoff, &indi.filt_cutoff)
PARAM_ADD(PARAM_FLOAT, filt_cutoff_r, &indi.filt_cutoff_r)
PARAM_ADD(PARAM_UINT8, outerLoopActive, &outerLoopActive)
PARAM_GROUP_STOP(ctrlINDI)

LOG_GROUP_START(ctrlINDI)
LOG_ADD(LOG_FLOAT, cmd_thrust, &indi.thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &indi.u_in.p)
LOG_ADD(LOG_FLOAT, cmd_pitch, &indi.u_in.q)
LOG_ADD(LOG_FLOAT, cmd_yaw, &indi.u_in.r)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, u_act_dyn.p, &indi.u_act_dyn.p)
LOG_ADD(LOG_FLOAT, u_act_dyn.q, &indi.u_act_dyn.q)
LOG_ADD(LOG_FLOAT, u_act_dyn.r, &indi.u_act_dyn.r)
LOG_ADD(LOG_FLOAT, du.p, &indi.du.p)
LOG_ADD(LOG_FLOAT, du.q, &indi.du.q)
LOG_ADD(LOG_FLOAT, du.r, &indi.du.r)
LOG_ADD(LOG_FLOAT, ang_accel_ref.p, &indi.angular_accel_ref.p)
LOG_ADD(LOG_FLOAT, ang_accel_ref.q, &indi.angular_accel_ref.q)
LOG_ADD(LOG_FLOAT, ang_accel_ref.r, &indi.angular_accel_ref.r)
LOG_ADD(LOG_FLOAT, rate_d[0], &indi.rate_d[0])
LOG_ADD(LOG_FLOAT, rate_d[1], &indi.rate_d[1])
LOG_ADD(LOG_FLOAT, rate_d[2], &indi.rate_d[2])
LOG_GROUP_STOP(ctrlINDI)
