#include "InertiaTensor.h"

/*
// This one is directly considering A is the center of gravity
float	ComputeInertiaTensor_BaseHalfbaseHeight(float b, float a, float h)
{
	float a3 = a * a * a;
	float a4 = a3 * a;
	float h3 = h * h * h;
	float b4 = b * b * b * b;

	float i1 = (a3 * h) / 4.0f + (a * h3) / 12.0f; // tensor of left rectangle triangle
	float i2 = (1.0f / (b - a)) * ((h * b4) / 12.0f - (a3 * h * b) / 3.0f + (a4 * h) / 4.0f + (h3 * (b - a) * (b - a)) / 12.0f); // tensor of right rectangle triangle

	return i1 + i2;
}

// A is being considered as the centered of gravity
float	ComputeInertiaTensor_Triangle(const Vec2& A, const Vec2& B, const Vec2& C)
{
Vec2 AB = B - A;
Vec2 AC = C - A;

float base, halfBase, height;
if (AB.GetSqrLength() >= AC.GetSqrLength())
{
base = AB.GetLength();
halfBase = AC | (AB / base);
height = (AC - AB * (halfBase / base)).GetLength();
}
else
{
base = AC.GetLength();
halfBase = AB | (AC / base);
height = (AB - AC * (halfBase / base)).GetLength();
}

return ComputeInertiaTensor_BaseHalfbaseHeight(base, halfBase, height);
}
*/

// inertia tensor from center of gravity
float ComputeInertiaTensor_BaseHalfbaseHeight(float b, float a, float h)
{
	float b2 = b * b;
	float b3 = b2 * b;
	float a2 = a * a;
	float a3 = a2 * a;
	float h3 = h * h * h;

	return (b3 * h - b2 * h * a + b * h *a2 + b * h3) * (1.0f / 36.0f);
}

// consider A is the center of gravity
float	ComputeInertiaTensor_Triangle(const Vec2& A, const Vec2& B, const Vec2& C)
{
	Vec2 AB = B - A;
	Vec2 AC = C - A;

	Vec2 G = (A + B + C) / 3.0f;

	float base, halfBase, height;
	if (AB.GetSqrLength() >= AC.GetSqrLength())
	{
		base = AB.GetLength();
		halfBase = AC | (AB / base);
		height = (AC - AB * (halfBase / base)).GetLength();
	}
	else
	{
		base = AC.GetLength();
		halfBase = AB | (AC / base);
		height = (AB - AC * (halfBase / base)).GetLength();
	}


	return ComputeInertiaTensor_BaseHalfbaseHeight(base, halfBase, height) + (G - A).GetSqrLength();
}


