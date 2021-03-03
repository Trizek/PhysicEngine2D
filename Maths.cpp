#include "Maths.h"

#include <stdlib.h>
#include <cmath>

float Sign(float a)
{
	return Select(a >= 0.0f, 1.0f, -1.0f);
}

float Random(float from, float to)
{
	return from + (to - from) * (((float)rand()) / ((float)RAND_MAX));
}


float ClampAngleRadians(float angle)
{
	return std::fmod(angle + (float)M_PI, (float)M_PI) - (float)M_PI;
}

Vec2 minv(const Vec2& a, const Vec2& b)
{
	return Vec2(Min(a.x, b.x), Min(a.y, b.y));
}

Vec2 maxv(const Vec2& a, const Vec2& b)
{
	return Vec2(Max(a.x, b.x), Max(a.y, b.y));
}

bool Clip(const Vec2& center, const Vec2& normal, Vec2& pt1, Vec2& pt2)
{
	float dist1 = (pt1 - center) | normal;
	float dist2 = (pt2 - center) | normal;

	if (dist1 * dist2 >= 0.0f)
	{
		return false;
	}

	Vec2 *pIn, *pOut;
	if (dist1 >= 0.0f)
	{
		pOut = &pt1;
		pIn = &pt2;
	}
	else
	{
		pOut = &pt2;
		pIn = &pt1;
	}

	float k = ((center - *pIn) | normal) / ((*pOut - *pIn) | normal);
	*pIn += (*pOut - *pIn) * k;

	return true;
}

// 2D Analytic LCP solver (find exact solution)
bool Solve2DLCP(const Mat2& A, const Mat2& invA, const Vec2& b, Vec2& x)
{
	// - We want :
	// y = A*x + b >= 0, transpose(x) * y = 0   
	//  x = (x1 x2) are the (new) accumulated impulses for contacts 1 and 2
	//	y = (y1 y2) are the contact relative velocity (minus the target/bias)
	// So we have x1 * y1 = x2 * y2 = 0  (for each contact, y > 0 means x = 0 (why impulse would participate if speed already higher than expected ?)
	//		and x > 0 means y = 0 (if impulse participate, it does it only to the extent needed))
	// - Therefore there 4 cases : x1 = x2 = 0, y1 = y2 = 0, x1 = y2 = 0, x2 = y1 = 0
	// - We compute each case and check if A*x + b >= 0, if so the LCP is solved
	// - We aim for stability, so we want the y to be 0 as much as possible, therefore we test first y1 = y2 = 0, then y1 = 0 and then y2 = 0
	Vec2 y;

	// case 1 : y =  A * x + b = (0 0)
	// x = inv(A) * (-b)
	x = invA * (b * -1.0f);
	if (x.x >= 0.0f && x.y >= 0.0f)
	{
		// x >= 0, all conditions are satisfied, we keep this impulse
		return true;
	}

	// case 2 : y1 = 0, x2 = 0 
	// A * (x1 0) + b = (0 y2)
	// a11 * x1 + b1 = 0   &&   a21 * x1 + b2 = y2
	// x1 = -b1 / a11
	// y2 = a21 * x1 + b2
	if (A.X.x != 0.0f)
	{
		x.x = -b.x / A.X.x;
		y.y = A.X.y * x.x + b.y;
		if (x.x >= 0.0f && y.y >= 0.0f)
		{
			x.y = 0.0f;
			return true;
		}
	}

	// case 2 : y2 = 0, x1 = 0 
	// A * (0 x2) + b = (y1 0)
	// a12 * x2 + b1 = y1  &&   a22 * x2 + b2 = 0
	// x2 = -b2 / a22
	// y1 = a12 * x2 + b1
	if (A.Y.x != 0.0f)
	{
		x.y = -b.y / A.Y.y;
		y.x = A.Y.x * x.y + b.x;
		if (x.x >= 0.0f && y.y >= 0.0f)
		{
			x.x = 0.0f;
			return true;
		}
	}

	// case 3 : x = (0 0) (no need extra testing, its the last possible case...)
	x.x = x.y = 0.0f;
	y = b;
	return (y.x >= 0.0f && y.y >= 0.0f);
}


float KernelDefault(float r, float h)
{
	float h2 = h * h;
	float h4 = h2 * h2;
	float kernel = h2 - r * r;
	return (kernel * kernel * kernel) * (4.0f / (((float)M_PI) * h4 * h4));
}

float KernelSpikyGradientFactorNorm(float r, float h)
{
	float h2 = h * h;
	float h5 = h2 * h2 * h;
	float kernel = h - r;
	return kernel * kernel * (-15.0f / ((float)M_PI * h5));
}

float KernelSpikyGradientFactor(float r, float h)
{
	float h2 = h * h;
	float h5 = h2 * h2 * h;
	float kernel = h - r;
	return kernel * kernel * (-15.0f / ((float)M_PI * h5 * r));
}

float KernelViscosityLaplacian(float r, float h)
{
	float h2 = h * h;
	float kernel = h - r;
	return kernel * (30.0f / ((float)M_PI * h2 * h2 * h));
}

float KernelPoly6hGradientFactor(float r, float h)
{
	float h2 = h * h;
	float kernel = h2 - r * r;
	return kernel * kernel * (24.0f / ((float)M_PI * h2 * h2 * h2 * h * r));
}