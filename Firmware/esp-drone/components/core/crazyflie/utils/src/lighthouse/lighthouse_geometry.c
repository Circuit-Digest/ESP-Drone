/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Code addapted from ashtuchkin's Vive DIY position sensor
 * see https://github.com/ashtuchkin/vive-diy-position-sensor/
 *
 * Copyright (c) 2016 Alexander Shtuchkin
 * Copyright (C) 2018 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouseGeometry.c: lighthouse tracking system geometry functions
 */

#include "lighthouse_geometry.h"

static void vec_cross_product(const vec3d a, const vec3d b, vec3d res) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

static float vec_dot(const vec3d a, const vec3d b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vec_add(const vec3d a, const vec3d b, vec3d r) {
  r[0] = a[0] + b[0];
  r[1] = a[1] + b[1];
  r[2] = a[2] + b[2];
}

static float vec_length(const vec3d vec) {
    float pow = vec_dot(vec, vec);

    float res;
    arm_sqrt_f32(pow, &res);
    return res;
}

static bool intersect_lines(vec3d orig1, vec3d vec1, vec3d orig2, vec3d vec2, vec3d res, float *dist) {
    // Algoritm: http://geomalgorithms.com/a07-_distance.html#Distance-between-Lines

    vec3d w0 = {};
    arm_sub_f32(orig1, orig2, w0, vec3d_size);

    float a, b, c, d, e;
    arm_dot_prod_f32(vec1, vec1, vec3d_size, &a);
    arm_dot_prod_f32(vec1, vec2, vec3d_size, &b);
    arm_dot_prod_f32(vec2, vec2, vec3d_size, &c);
    arm_dot_prod_f32(vec1, w0, vec3d_size, &d);
    arm_dot_prod_f32(vec2, w0, vec3d_size, &e);

    float denom = a * c - b * b;
    if (fabsf(denom) < 1e-5f)
        return false;

    // Closest point to 2nd line on 1st line
    float t1 = (b * e - c * d) / denom;
    vec3d pt1 = {};
    arm_scale_f32(vec1, t1, pt1, vec3d_size);
    arm_add_f32(pt1, orig1, pt1, vec3d_size);

    // Closest point to 1st line on 2nd line
    float t2 = (a * e - b * d) / denom;
    vec3d pt2 = {};
    arm_scale_f32(vec2, t2, pt2, vec3d_size);
    arm_add_f32(pt2, orig2, pt2, vec3d_size);

    // Result is in the middle
    vec3d tmp = {};
    arm_add_f32(pt1, pt2, tmp, vec3d_size);
    arm_scale_f32(tmp, 0.5f, res, vec3d_size);

    // Dist is distance between pt1 and pt2
    arm_sub_f32(pt1, pt2, tmp, vec3d_size);
    *dist = vec_length(tmp);

    return true;
}

bool lighthouseGeometryGetPositionFromRayIntersection(baseStationGeometry_t baseStations[2], float angles1[2], float angles2[2], vec3d position, float *position_delta)
{
    static vec3d ray1, ray2, origin1, origin2;

    lighthouseGeometryGetRay(&baseStations[0], angles1[0], angles1[1], ray1);
    lighthouseGeometryGetBaseStationPosition(&baseStations[0], origin1);

    lighthouseGeometryGetRay(&baseStations[1], angles2[0], angles2[1], ray2);
    lighthouseGeometryGetBaseStationPosition(&baseStations[1], origin2);

    return intersect_lines(origin1, ray1, origin2, ray2, position, position_delta);
}

void lighthouseGeometryGetBaseStationPosition(baseStationGeometry_t* bs, vec3d baseStationPos) {
    // TODO: Make geometry adjustments within base station.
    vec3d rotated_origin_delta = {};
    //vec3d base_origin_delta = {-0.025f, -0.025f, 0.f};  // Rotors are slightly off center in base station.
    // arm_matrix_instance_f32 origin_vec = {3, 1, base_origin_delta};
    // arm_matrix_instance_f32 origin_rotated_vec = {3, 1, rotated_origin_delta};
    // arm_mat_mult_f32(&source_rotation_matrix, &origin_vec, &origin_rotated_vec);
    arm_add_f32(bs->origin, rotated_origin_delta, baseStationPos, vec3d_size);
}

void lighthouseGeometryGetRay(const baseStationGeometry_t* baseStationGeometry, const float angleH, const float angleV, vec3d ray) {
    vec3d a = {arm_sin_f32(angleH), -arm_cos_f32(angleH), 0};  // Normal vector to X plane
    vec3d b = {-arm_sin_f32(angleV), 0, arm_cos_f32(angleV)};  // Normal vector to Y plane

    vec3d raw_ray = {};
    vec_cross_product(b, a, raw_ray); // Intersection of two planes -> ray vector.
    float len = vec_length(raw_ray);
    arm_scale_f32(raw_ray, 1 / len, raw_ray, vec3d_size); // Normalize raw ray length.

    arm_matrix_instance_f32 source_rotation_matrix = {3, 3, (float32_t *)baseStationGeometry->mat};
    arm_matrix_instance_f32 ray_vec = {3, 1, raw_ray};
    arm_matrix_instance_f32 ray_rotated_vec = {3, 1, ray};
    arm_mat_mult_f32(&source_rotation_matrix, &ray_vec, &ray_rotated_vec);
}

bool lighthouseGeometryIntersectionPlaneVector(const vec3d linePoint, const vec3d lineVec, const vec3d planePoint, const vec3d PlaneNormal, vec3d intersectionPoint) {
    float p = -vec_dot(lineVec, PlaneNormal);
    if (p == 0.0f) {
        // Line and plane is parallel, no intersection point
        return false;
    }

    vec3d lp = {linePoint[0] - planePoint[0], linePoint[1] - planePoint[1], linePoint[2] - planePoint[2]};
    float t = vec_dot(PlaneNormal, lp) / p;

    intersectionPoint[0] = linePoint[0] + lineVec[0] * t;
    intersectionPoint[1] = linePoint[1] + lineVec[1] * t;
    intersectionPoint[2] = linePoint[2] + lineVec[2] * t;

    return true;
}

void lighthouseGeometryGetSensorPosition(const vec3d cfPos, const arm_matrix_instance_f32 *R, vec3d sensorPosition, vec3d pos) {
  arm_matrix_instance_f32 LOCAL_POS = {3, 1, sensorPosition};

  vec3d rotatedPos = {0};
  arm_matrix_instance_f32 ROTATED_POS = {1, 3, rotatedPos};
  arm_mat_mult_f32(R, &LOCAL_POS, &ROTATED_POS);

  vec_add(cfPos, rotatedPos, pos);
}

bool lighthouseGeometryYawDelta(const vec3d ipv, const vec3d spv, const vec3d n, float* yawDelta) {
    float spvn = vec_length(spv);
    float ipvn = vec_length(ipv);

    const float minIntersectionVectorLength = 0.0001f;
    if (ipvn < minIntersectionVectorLength) {
        return false;
    }

    // Angle between vectors
    float a = vec_dot(spv, ipv) / (spvn * ipvn);

    // Handle rouding errors
    if (a > 1.0f) {
        a = 1.0f;
    }
    if (a < -1.0f) {
        a = -1.0f;
    }

    float delta = acosf(a);

    // Figure our the sign of the angle by looking at the cross product of the vectors in relation to the normal
    float nt[3];
    vec_cross_product(spv, ipv, nt);

    if (vec_dot(n, nt) > 0.0f) {
        delta = -delta;
    }

    *yawDelta = delta;
    return true;
}

void lighthouseGeometryCalculateAnglesFromRotationMatrix(baseStationGeometry_t* baseStationGeometry, baseStationEulerAngles_t* baseStationEulerAngles) {

   /*
    * roll pitch yaw Rotation matrix
    *
    R = | R00 R01 R02 |
        | R10 R11 R12 |
        | R20 R21 R22 |
        */

  float32_t R00 = (float32_t)baseStationGeometry->mat[0][0];
  float32_t R01 = (float32_t)baseStationGeometry->mat[0][1];
  float32_t R02 = (float32_t)baseStationGeometry->mat[0][2];
  float32_t R12 = (float32_t)baseStationGeometry->mat[1][2];
  float32_t R22 = (float32_t)baseStationGeometry->mat[2][2];


  float pitchBaseStation = asin(R02);
  float yawBaseStation = -1.0*atan2(R01,R00);
  float rollBaseStation = -1.0*atan2(R12,R22);

  baseStationEulerAngles->roll=rollBaseStation;
  baseStationEulerAngles->pitch=pitchBaseStation;
  baseStationEulerAngles->yaw=yawBaseStation;

}
