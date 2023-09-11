/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Evghenii Volodscoi
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
 * controller to control the position of the Crazyflie. It can be seen as an extension
 * (outer loop) to the already existing INDI attitude controller (inner loop).
 * 
 * The control algorithm was implemented according to the publication in the
 * journal od Control Engineering Practice: Cascaded Incremental Nonlinear Dynamic 
 * Inversion for MAV Disturbance Rejection
 * https://doi.org/10.1016/j.conengprac.2018.01.003
 */


#include "position_controller_indi.h"
#include "math3d.h"

// Position controller gains
float K_xi_x = 1.0f;
float K_xi_y = 1.0f;
float K_xi_z = 1.0f;
// Velocity controller gains
float K_dxi_x = 5.0f;
float K_dxi_y = 5.0f;
float K_dxi_z = 5.0f;
// Thrust mapping parameter
float K_thr = 0.00024730f;

static float posS_x, posS_y, posS_z;			// Current position
static float velS_x, velS_y, velS_z;			// Current velocity
static float gyr_p, gyr_q, gyr_r;				// Current rates

// Reference values
static struct Vectr positionRef; 
static struct Vectr velocityRef;


static struct IndiOuterVariables indiOuter = {
	.filt_cutoff = POSITION_INDI_FILT_CUTOFF,
	.act_dyn_posINDI = STABILIZATION_INDI_ACT_DYN_P
};


void position_indi_init_filters(void)
{
	// tau = 1/(2*pi*Fc)
	float tau = 1.0f / (2.0f * M_PI_F * indiOuter.filt_cutoff);
	float tau_axis[3] = {tau, tau, tau};
	float sample_time = 1.0f / ATTITUDE_RATE;
	// Filtering of linear acceleration, attitude and thrust 
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&indiOuter.ddxi[i], tau_axis[i], sample_time, 0.0f);
		init_butterworth_2_low_pass(&indiOuter.ang[i], tau_axis[i], sample_time, 0.0f);
		init_butterworth_2_low_pass(&indiOuter.thr[i], tau_axis[i], sample_time, 0.0f);
	}
}

// Linear acceleration filter
static inline void filter_ddxi(Butterworth2LowPass *filter, struct Vectr *old_values, struct Vectr *new_values)
{
	new_values->x = update_butterworth_2_low_pass(&filter[0], old_values->x);
	new_values->y = update_butterworth_2_low_pass(&filter[1], old_values->y);
	new_values->z = update_butterworth_2_low_pass(&filter[2], old_values->z);
}

// Attitude filter
static inline void filter_ang(Butterworth2LowPass *filter, struct Angles *old_values, struct Angles *new_values)
{
	new_values->phi = update_butterworth_2_low_pass(&filter[0], old_values->phi);
	new_values->theta = update_butterworth_2_low_pass(&filter[1], old_values->theta);
	new_values->psi = update_butterworth_2_low_pass(&filter[2], old_values->psi);
}

// Thrust filter
static inline void filter_thrust(Butterworth2LowPass *filter, float *old_thrust, float *new_thrust) 
{
	*new_thrust = update_butterworth_2_low_pass(&filter[0], *old_thrust);
}


// Computes transformation matrix from body frame (index B) into NED frame (index O)
void m_ob(struct Angles att, float matrix[3][3]) {

	matrix[0][0] = cosf(att.theta)*cosf(att.psi);
	matrix[0][1] = sinf(att.phi)*sinf(att.theta)*cosf(att.psi) - cosf(att.phi)*sinf(att.psi); 
	matrix[0][2] = cosf(att.phi)*sinf(att.theta)*cosf(att.psi) + sinf(att.phi)*sinf(att.psi);
	matrix[1][0] = cosf(att.theta)*sinf(att.psi);
	matrix[1][1] = sinf(att.phi)*sinf(att.theta)*sinf(att.psi) + cosf(att.phi)*cosf(att.psi);
	matrix[1][2] = cosf(att.phi)*sinf(att.theta)*sinf(att.psi) - sinf(att.phi)*cosf(att.psi);
	matrix[2][0] = -sinf(att.theta);
	matrix[2][1] = sinf(att.phi)*cosf(att.theta);
	matrix[2][2] = cosf(att.phi)*cosf(att.theta);
}


void positionControllerINDIInit(void)
{
	// Re-initialize filters
	position_indi_init_filters();
}


void positionControllerINDI(const sensorData_t *sensors,
                            setpoint_t *setpoint,
                            const state_t *state, 
                            vector_t *refOuterINDI){ 

	// Read states (position, velocity)
	posS_x = state->position.x;
	posS_y = -state->position.y;
	posS_z = -state->position.z;
	velS_x = state->velocity.x;
	velS_y = -state->velocity.y;
	velS_z = -state->velocity.z;
	gyr_p = sensors->gyro.x;
	gyr_q = sensors->gyro.y;
	gyr_r = sensors->gyro.z; 

	// Read in velocity setpoints
    velocityRef.x = setpoint->velocity.x;
	velocityRef.y = -setpoint->velocity.y;
    velocityRef.z = -setpoint->velocity.z;

	// Position controller (Proportional)
	if (setpoint->mode.x == modeAbs) {
		positionRef.x = setpoint->position.x;
		velocityRef.x = K_xi_x*(positionRef.x - posS_x);
	}
	if (setpoint->mode.y == modeAbs) {
		positionRef.y = -setpoint->position.y;
		velocityRef.y = K_xi_y*(positionRef.y - posS_y);
	}
	if (setpoint->mode.z == modeAbs) {
		positionRef.z = -setpoint->position.z;
		velocityRef.z = K_xi_z*(positionRef.z - posS_z);
	}

	// Velocity controller (Proportional)
	indiOuter.linear_accel_ref.x = K_dxi_x*(velocityRef.x - velS_x);
	indiOuter.linear_accel_ref.y = K_dxi_y*(velocityRef.y - velS_y);
	indiOuter.linear_accel_ref.z = K_dxi_z*(velocityRef.z - velS_z); 

	// Acceleration controller (INDI)
	// Read lin. acceleration (Body-fixed) obtained from sensors CHECKED
	indiOuter.linear_accel_s.x = (sensors->acc.x)*9.81f;
	indiOuter.linear_accel_s.y = (-sensors->acc.y)*9.81f;
	indiOuter.linear_accel_s.z = (-sensors->acc.z)*9.81f;

	// Filter lin. acceleration 
	filter_ddxi(indiOuter.ddxi, &indiOuter.linear_accel_s, &indiOuter.linear_accel_f);

	// Obtain actual attitude values (in deg)
	indiOuter.attitude_s.phi = state->attitude.roll; 
	indiOuter.attitude_s.theta = state->attitude.pitch;
	indiOuter.attitude_s.psi = -state->attitude.yaw;
	filter_ang(indiOuter.ang, &indiOuter.attitude_s, &indiOuter.attitude_f);


	// Actual attitude (in rad)
	struct Angles att = {
		.phi = indiOuter.attitude_f.phi/180*M_PI_F,
		.theta = indiOuter.attitude_f.theta/180*M_PI_F,
		.psi = indiOuter.attitude_f.psi/180*M_PI_F,
	};

	// Compute transformation matrix from body frame (index B) into NED frame (index O)
	float M_OB[3][3] = {0};
	m_ob(att, M_OB);

	// Transform lin. acceleration in NED (add gravity to the z-component)
	indiOuter.linear_accel_ft.x = M_OB[0][0]*indiOuter.linear_accel_f.x + M_OB[0][1]*indiOuter.linear_accel_f.y + M_OB[0][2]*indiOuter.linear_accel_f.z;
	indiOuter.linear_accel_ft.y = M_OB[1][0]*indiOuter.linear_accel_f.x + M_OB[1][1]*indiOuter.linear_accel_f.y + M_OB[1][2]*indiOuter.linear_accel_f.z;
	indiOuter.linear_accel_ft.z = M_OB[2][0]*indiOuter.linear_accel_f.x + M_OB[2][1]*indiOuter.linear_accel_f.y + M_OB[2][2]*indiOuter.linear_accel_f.z + 9.81f;

	// Compute lin. acceleration error
	indiOuter.linear_accel_err.x = indiOuter.linear_accel_ref.x - indiOuter.linear_accel_ft.x;
	indiOuter.linear_accel_err.y = indiOuter.linear_accel_ref.y - indiOuter.linear_accel_ft.y;
	indiOuter.linear_accel_err.z = indiOuter.linear_accel_ref.z - indiOuter.linear_accel_ft.z;

	// Elements of the G matrix (see publication for more information) 
	// ("-" because T points in neg. z-direction, "*9.81" because T/m=a=g, 
	// negative psi to account for wrong coordinate frame in the implementation of the inner loop)
	float g11 = (cosf(att.phi)*sinf(-att.psi) - sinf(att.phi)*sinf(att.theta)*cosf(-att.psi))*(-9.81f);
	float g12 = (cosf(att.phi)*cosf(-att.theta)*cosf(-att.psi))*(-9.81f);
	float g13 = (sinf(att.phi)*sinf(-att.psi) + cosf(att.phi)*sinf(att.theta)*cosf(-att.psi));
	float g21 = (-cosf(att.phi)*cosf(-att.psi) - sinf(att.phi)*sinf(att.theta)*sinf(-att.psi))*(-9.81f);
	float g22 = (cosf(att.phi)*cosf(att.theta)*sinf(-att.psi))*(-9.81f);
	float g23 = (-sinf(att.phi)*cosf(-att.psi) + cosf(att.phi)*sinf(att.theta)*sinf(-att.psi));
	float g31 = (-sinf(att.phi)*cosf(att.theta))*(-9.81f);
	float g32 = (-cosf(att.phi)*sinf(att.theta))*(-9.81f);
	float g33 = (cosf(att.phi)*cosf(att.theta));

	// Next four blocks of the code are to compute the Moore-Penrose inverse of the G matrix
	// (G'*G)
	float a11 = g11*g11 + g21*g21 + g31*g31;
	float a12 = g11*g12 + g21*g22 + g31*g32;
	float a13 = g11*g13 + g21*g23 + g31*g33;
	float a21 = g12*g11 + g22*g21 + g32*g31;
	float a22 = g12*g12 + g22*g22 + g32*g32;
	float a23 = g12*g13 + g22*g23 + g32*g33;
	float a31 = g13*g11 + g23*g21 + g33*g31;
	float a32 = g13*g12 + g23*g22 + g33*g32;
	float a33 = g13*g13 + g23*g23 + g33*g33;

	// Determinant of (G'*G)
	float detG = (a11*a22*a33 + a12*a23*a31 + a21*a32*a13) - (a13*a22*a31 + a11*a32*a23 + a12*a21*a33); 

	// Inverse of (G'*G)
	float a11_inv = (a22*a33 - a23*a32)/detG;
	float a12_inv = (a13*a32 - a12*a33)/detG;
	float a13_inv = (a12*a23 - a13*a22)/detG;
	float a21_inv = (a23*a31 - a21*a33)/detG;
	float a22_inv = (a11*a33 - a13*a31)/detG;
	float a23_inv = (a13*a21 - a11*a23)/detG;
	float a31_inv = (a21*a32 - a22*a31)/detG;
	float a32_inv = (a12*a31 - a11*a32)/detG;
	float a33_inv = (a11*a22 - a12*a21)/detG; 

	// G_inv = (G'*G)_inv*G'
	float g11_inv = a11_inv*g11 + a12_inv*g12 + a13_inv*g13;
	float g12_inv = a11_inv*g21 + a12_inv*g22 + a13_inv*g23;
	float g13_inv = a11_inv*g31 + a12_inv*g32 + a13_inv*g33;
	float g21_inv = a21_inv*g11 + a22_inv*g12 + a23_inv*g13;
	float g22_inv = a21_inv*g21 + a22_inv*g22 + a23_inv*g23;
	float g23_inv = a21_inv*g31 + a22_inv*g32 + a23_inv*g33;
	float g31_inv = a31_inv*g11 + a32_inv*g12 + a33_inv*g13;
	float g32_inv = a31_inv*g21 + a32_inv*g22 + a33_inv*g23;
	float g33_inv = a31_inv*g31 + a32_inv*g32 + a33_inv*g33;

	// Lin. accel. error multiplied  G^(-1) matrix (T_tilde negated because motor accepts only positiv commands, angles are in rad)
	indiOuter.phi_tilde   = (g11_inv*indiOuter.linear_accel_err.x + g12_inv*indiOuter.linear_accel_err.y + g13_inv*indiOuter.linear_accel_err.z);
	indiOuter.theta_tilde = (g21_inv*indiOuter.linear_accel_err.x + g22_inv*indiOuter.linear_accel_err.y + g23_inv*indiOuter.linear_accel_err.z);
	indiOuter.T_tilde     = -(g31_inv*indiOuter.linear_accel_err.x + g32_inv*indiOuter.linear_accel_err.y + g33_inv*indiOuter.linear_accel_err.z)/K_thr; 	

	// Filter thrust
	filter_thrust(indiOuter.thr, &indiOuter.T_incremented, &indiOuter.T_inner_f);

	// Pass thrust through the model of the actuator dynamics
	indiOuter.T_inner = indiOuter.T_inner + indiOuter.act_dyn_posINDI*(indiOuter.T_inner_f - indiOuter.T_inner); 

	// Compute trust that goes into the inner loop
		indiOuter.T_incremented = indiOuter.T_tilde + indiOuter.T_inner;

	// Compute commanded attitude to the inner INDI
	indiOuter.attitude_c.phi = indiOuter.attitude_f.phi + indiOuter.phi_tilde*180/M_PI_F;
	indiOuter.attitude_c.theta = indiOuter.attitude_f.theta + indiOuter.theta_tilde*180/M_PI_F;

	// Clamp commands
	indiOuter.T_incremented = clamp(indiOuter.T_incremented, MIN_THRUST, MAX_THRUST);
	indiOuter.attitude_c.phi = clamp(indiOuter.attitude_c.phi, -10.0f, 10.0f);
	indiOuter.attitude_c.theta = clamp(indiOuter.attitude_c.theta, -10.0f, 10.0f);

	// Reference values, which are passed to the inner loop INDI (attitude controller)
	refOuterINDI->x = indiOuter.attitude_c.phi;
	refOuterINDI->y = indiOuter.attitude_c.theta;
	refOuterINDI->z = indiOuter.T_incremented;

}


PARAM_GROUP_START(posCtrlIndi)

// Position controller gain
PARAM_ADD(PARAM_FLOAT, K_xi_x, &K_xi_x)
PARAM_ADD(PARAM_FLOAT, K_xi_y, &K_xi_y)
PARAM_ADD(PARAM_FLOAT, K_xi_z, &K_xi_z)

// Velocity Controller gain
PARAM_ADD(PARAM_FLOAT, K_dxi_x, &K_dxi_x)
PARAM_ADD(PARAM_FLOAT, K_dxi_y, &K_dxi_y)
PARAM_ADD(PARAM_FLOAT, K_dxi_z, &K_dxi_z)

PARAM_GROUP_STOP(posCtrlIndi)



LOG_GROUP_START(posCtrlIndi)

// Angular veocity
LOG_ADD(LOG_FLOAT, gyr_p, &gyr_p)
LOG_ADD(LOG_FLOAT, gyr_q, &gyr_q)
LOG_ADD(LOG_FLOAT, gyr_r, &gyr_r)

LOG_ADD(LOG_FLOAT, posRef_x, &positionRef.x)
LOG_ADD(LOG_FLOAT, posRef_y, &positionRef.y)
LOG_ADD(LOG_FLOAT, posRef_z, &positionRef.z)

// Velocity
LOG_ADD(LOG_FLOAT, velS_x, &velS_x)
LOG_ADD(LOG_FLOAT, velS_y, &velS_y)
LOG_ADD(LOG_FLOAT, velS_z, &velS_z)

LOG_ADD(LOG_FLOAT, velRef_x, &velocityRef.x)
LOG_ADD(LOG_FLOAT, velRef_y, &velocityRef.y)
LOG_ADD(LOG_FLOAT, velRef_z, &velocityRef.z)

// Attitude
LOG_ADD(LOG_FLOAT, angS_roll, &indiOuter.attitude_s.phi)
LOG_ADD(LOG_FLOAT, angS_pitch, &indiOuter.attitude_s.theta)
LOG_ADD(LOG_FLOAT, angS_yaw, &indiOuter.attitude_s.psi)

LOG_ADD(LOG_FLOAT, angF_roll, &indiOuter.attitude_f.phi)
LOG_ADD(LOG_FLOAT, angF_pitch, &indiOuter.attitude_f.theta)
LOG_ADD(LOG_FLOAT, angF_yaw, &indiOuter.attitude_f.psi)

// Acceleration
LOG_ADD(LOG_FLOAT, accRef_x, &indiOuter.linear_accel_ref.x)
LOG_ADD(LOG_FLOAT, accRef_y, &indiOuter.linear_accel_ref.y)
LOG_ADD(LOG_FLOAT, accRef_z, &indiOuter.linear_accel_ref.z)

LOG_ADD(LOG_FLOAT, accS_x, &indiOuter.linear_accel_s.x)
LOG_ADD(LOG_FLOAT, accS_y, &indiOuter.linear_accel_s.y)
LOG_ADD(LOG_FLOAT, accS_z, &indiOuter.linear_accel_s.z)

LOG_ADD(LOG_FLOAT, accF_x, &indiOuter.linear_accel_f.x)
LOG_ADD(LOG_FLOAT, accF_y, &indiOuter.linear_accel_f.y)
LOG_ADD(LOG_FLOAT, accF_z, &indiOuter.linear_accel_f.z)

LOG_ADD(LOG_FLOAT, accFT_x, &indiOuter.linear_accel_ft.x)
LOG_ADD(LOG_FLOAT, accFT_y, &indiOuter.linear_accel_ft.y)
LOG_ADD(LOG_FLOAT, accFT_z, &indiOuter.linear_accel_ft.z)

LOG_ADD(LOG_FLOAT, accErr_x, &indiOuter.linear_accel_err.x)
LOG_ADD(LOG_FLOAT, accErr_y, &indiOuter.linear_accel_err.y)
LOG_ADD(LOG_FLOAT, accErr_z, &indiOuter.linear_accel_err.z)

// INDI outer loop variables
LOG_ADD(LOG_FLOAT, phi_tilde, &indiOuter.phi_tilde)
LOG_ADD(LOG_FLOAT, theta_tilde, &indiOuter.theta_tilde)
LOG_ADD(LOG_FLOAT, T_tilde, &indiOuter.T_tilde)

LOG_ADD(LOG_FLOAT, T_inner, &indiOuter.T_inner)
LOG_ADD(LOG_FLOAT, T_inner_f, &indiOuter.T_inner_f)
LOG_ADD(LOG_FLOAT, T_incremented, &indiOuter.T_incremented)

LOG_ADD(LOG_FLOAT, cmd_phi, &indiOuter.attitude_c.phi)
LOG_ADD(LOG_FLOAT, cmd_theta, &indiOuter.attitude_c.theta)

LOG_GROUP_STOP(posCtrlIndi)
