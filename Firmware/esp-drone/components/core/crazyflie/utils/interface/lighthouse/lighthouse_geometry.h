#pragma once

#include <stdbool.h>
#include "xtensa_math.h"
#include "stabilizer_types.h"

typedef struct {
  __attribute__((aligned(4))) vec3d origin;
  __attribute__((aligned(4))) mat3d mat;
  bool valid;
} __attribute__((packed)) baseStationGeometry_t;

typedef struct {
  __attribute__((aligned(4))) mat3d baseStationInvertedRotationMatrixes;
  __attribute__((aligned(4))) mat3d lh1Rotor2RotationMatrixes;
  __attribute__((aligned(4))) mat3d lh1Rotor2InvertedRotationMatrixes;
} baseStationGeometryCache_t;

/**
 * @brief Find closest point between rays from two bases stations.
 *
 * @param baseStations - Geometry data for the two base statsions (position and orientation)
 * @param angles1 - array with 2 angles, horizontal and vertical sweep angle for base station 1
 * @param angles2 - array with 2 angles, horizontal and vertical sweep angle for base station 2
 * @param position - (output) the closest point between the rays
 * @param postion_delta - (output) the distance between the rays at the closest point
 */
bool lighthouseGeometryGetPositionFromRayIntersection(const baseStationGeometry_t baseStations[2], float angles1[2], float angles2[2], vec3d position, float *position_delta);

/**
 * @brief Get the base station position from the base station geometry in world reference frame. This position can be seen as the
 * point where the lazers originate from.
 *
 * @param baseStation - Geometry data for the base statsion (position and orientation)
 * @param baseStationPos - (output) the base station position
 */
void lighthouseGeometryGetBaseStationPosition(const baseStationGeometry_t* baseStationGeometry, vec3d baseStationPos);

/**
 * @brief Get a normalized vector representing the direction of a ray in world reference frame, based on
 * sweep angles and base station orientation.
 *
 * @param baseStation - Geometry data for the base statsion (position and orientation)
 * @param angle1 - horizontal sweep angle
 * @param angle2 - vertical sweep angle
 * @param ray - (output) the resulting normalized vector
 */
void lighthouseGeometryGetRay(const baseStationGeometry_t* baseStationGeometry, const float angle1, const float angle2, vec3d ray);

/**
 * @brief Calculates the intersection point between a plane and a line
 *
 * @param linePoint - a point on the line
 * @param lineVec - a normalized vector in the direction of the line
 * @param planePoint - a point in the plane
 * @param PlaneNormal - the normal to the plane (normalized)
 * @param intersectionPoint - (output) the intersection point
 * @return true if an intersection exists
 */
bool lighthouseGeometryIntersectionPlaneVector(const vec3d linePoint, const vec3d lineVec, const vec3d planePoint, const vec3d PlaneNormal, vec3d intersectionPoint);

/**
 * @brief Calculate the position of a sensor on the deck in world reference frame.
 * Note: the origin of the Crazyflie is set in the center of the deck
 *
 * @param cfPos - Crazyflie position
 * @param R - Crazyflie rotation matrix
 * @param sensorPosition - the sensor position relative to the center of the Crazyflie
 * @param pos - (output) the position of the sensor
 */
void lighthouseGeometryGetSensorPosition(const vec3d cfPos, const arm_matrix_instance_f32 *R, vec3d sensorPosition, vec3d pos);

/**
 * @brief Calculate the angle between two vectors. The vectors are assumed to be in a plane defined by
 * a normal and represent the distance between intersection points and sensor points on a lighthouse deck.
 *
 * @param ipv - vector between intersection points
 * @param spv - vector between sensor points
 * @param n - normal to the plane (normalized)
 * @param yawDelta - (output) the angle between the vectors
 *
 * @return true if the angle could be calculated
*/
bool lighthouseGeometryYawDelta(const vec3d ipv, const vec3d spv, const vec3d n, float* yawDelta);
