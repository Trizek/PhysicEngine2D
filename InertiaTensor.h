#ifndef _INERTIA_TENSOR_H_
#define _INERTIA_TENSOR_H_

#include "Maths.h"

// A is being considered as the centered of gravity
float	ComputeInertiaTensor_BaseHalfbaseHeight(float b, float a, float h);

// consider A is the center of gravity
float	ComputeInertiaTensor_Triangle(const Vec2& A, const Vec2& B, const Vec2& C);




#endif