/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D'Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D'Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 */

#ifndef __ESTIMATOR_KALMAN_H__
#define __ESTIMATOR_KALMAN_H__

#include <stdint.h>
#include "stabilizer_types.h"

void estimatorKalmanInit(void);
bool estimatorKalmanTest(void);
void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);


void estimatorKalmanTaskInit();
bool estimatorKalmanTaskTest();

/**
 * The filter supports the incorporation of additional sensors into the state estimate via the following functions:
 */
bool estimatorKalmanEnqueueTDOA(const tdoaMeasurement_t *uwb);
bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos);
bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose);
bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist);
bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof);
bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height);
bool estimatorKalmanEnqueueFlow(const flowMeasurement_t *flow);
bool estimatorKalmanEnqueueYawError(const yawErrorMeasurement_t* error);
//bool estimatorKalmanEnqueueSweepAngles(const sweepAngleMeasurement_t *angles);

void estimatorKalmanGetEstimatedPos(point_t* pos);

/**
 * Copies 9 floats representing the current state rotation matrix
 */
void estimatorKalmanGetEstimatedRot(float * rotationMatrix);

#endif // __ESTIMATOR_KALMAN_H__
