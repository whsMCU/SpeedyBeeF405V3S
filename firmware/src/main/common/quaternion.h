/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Original implementation by HaukeRa
 * Refactored and adapted by DigitalEntity
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include "common/maths.h"
#include "common/vector.h"

#include "flight/imu.h"

typedef struct {
    float q0, q1, q2, q3;
} fpQuaternion_t;

static inline fpQuaternion_t * quaternionInitUnit(fpQuaternion_t * result)
{
    result->q0 = 1.0f;
    result->q1 = 0.0f;
    result->q2 = 0.0f;
    result->q3 = 0.0f;
    return result;
}

static inline fpQuaternion_t * quaternionInitFromVector(fpQuaternion_t * result, const fpVector3_t * v)
{
    result->q0 = 0.0f;
    result->q1 = v->x;
    result->q2 = v->y;
    result->q3 = v->z;
    return result;
}

static inline void quaternionToAxisAngle(fpAxisAngle_t * result, const fpQuaternion_t * q)
{
    fpAxisAngle_t a = {.axis = {{1.0f, 0.0f, 0.0f}}, .angle = 0};

    a.angle = 2.0f * acos_approx(constrainf(q->q0, -1.0f, 1.0f));

    if (a.angle > M_PIf) {
        a.angle -= 2.0f * M_PIf;
    }

    const float sinVal = sqrt(1.0f - q->q0 * q->q0);

    // Axis is only valid when rotation is large enough sin(0.0057 deg) = 0.0001
    if (sinVal > 1e-4f) {
        a.axis.x = q->q1 / sinVal;
        a.axis.y = q->q2 / sinVal;
        a.axis.z = q->q3 / sinVal;
    } else {
        a.angle = 0;
    }

    *result = a;
}

static inline fpQuaternion_t * axisAngleToQuaternion(fpQuaternion_t * result, const fpAxisAngle_t * a)
{
  fpQuaternion_t q;
  const float s = sin_approx(a->angle / 2.0f);

  q.q0 = cos_approx(a->angle / 2.0f);
  q.q1 = -a->axis.x * s;
  q.q2 = -a->axis.y * s;
  q.q3 = -a->axis.z * s;

  *result = q;
  return result;
}

static inline float quaternionNormSqared(const fpQuaternion_t * q)
{
    return sq(q->q0) + sq(q->q1) + sq(q->q2) + sq(q->q3);
}

static inline quaternion * quaternionMultiply(quaternion * result, const quaternion * a, const quaternion * b)
{
  quaternion p;

  p.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
  p.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
  p.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
  p.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;

  *result = p;
  return result;
}

static inline fpQuaternion_t * quaternionScale(fpQuaternion_t * result, const fpQuaternion_t * a, const float b)
{
    fpQuaternion_t p;

    p.q0 = a->q0 * b;
    p.q1 = a->q1 * b;
    p.q2 = a->q2 * b;
    p.q3 = a->q3 * b;

    *result = p;
    return result;
}

static inline fpQuaternion_t * quaternionAdd(fpQuaternion_t * result, const fpQuaternion_t * a, const fpQuaternion_t * b)
{
    fpQuaternion_t p;

    p.q0 = a->q0 + b->q0;
    p.q1 = a->q1 + b->q1;
    p.q2 = a->q2 + b->q2;
    p.q3 = a->q3 + b->q3;

    *result = p;
    return result;
}

static inline quaternion * quaternionConjugate(quaternion * result, const quaternion * q)
{
    result->w =  q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;

    return result;
}

static inline fpQuaternion_t * quaternionNormalize(fpQuaternion_t * result, const fpQuaternion_t * q)
{
    float mod = fast_fsqrtf(quaternionNormSqared(q));
    if (mod < 1e-6f) {
        // Length is too small - re-initialize to zero rotation
        result->q0 = 1;
        result->q1 = 0;
        result->q2 = 0;
        result->q3 = 0;
    }
    else {
        result->q0 = q->q0 / mod;
        result->q1 = q->q1 / mod;
        result->q2 = q->q2 / mod;
        result->q3 = q->q3 / mod;
    }

    return result;
}

static inline fpVector3_t * quaternionRotateVector(fpVector3_t * result, const fpVector3_t * vect, const quaternion * ref)
{
  quaternion vectQuat, refConj;

    vectQuat.w = 0;
    vectQuat.x = vect->x;
    vectQuat.y = vect->y;
    vectQuat.z = vect->z;

    quaternionConjugate(&refConj, ref);
    quaternionMultiply(&vectQuat, &refConj, &vectQuat);
    quaternionMultiply(&vectQuat, &vectQuat, ref);

    result->x = vectQuat.x;
    result->y = vectQuat.y;
    result->z = vectQuat.z;
    return result;
}

static inline fpVector3_t * quaternionRotateVectorInv(fpVector3_t * result, const fpVector3_t * vect, const quaternion * ref)
{
  quaternion vectQuat, refConj;

    vectQuat.w = 0;
    vectQuat.x = vect->x;
    vectQuat.y = vect->y;
    vectQuat.z = vect->z;

    quaternionConjugate(&refConj, ref);
    quaternionMultiply(&vectQuat, ref, &vectQuat);
    quaternionMultiply(&vectQuat, &vectQuat, &refConj);

    result->x = vectQuat.x;
    result->y = vectQuat.y;
    result->z = vectQuat.z;
    return result;
}
