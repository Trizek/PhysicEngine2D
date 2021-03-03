#ifndef _COLLISION_H_
#define _COLLISION_H_

#include "Polygon.h"

struct SPolygonPair
{
	SPolygonPair(CPolygonPtr _polyA, CPolygonPtr _polyB) : polyA(_polyA), polyB(_polyB){}

	CPolygonPtr	polyA;
	CPolygonPtr	polyB;
};

struct SContactInfo
{
	SContactInfo() = default;
	SContactInfo(CPolygon* _pA, CPolygon* _pB, const Vec2& _pt, const Vec2& _normal, float _penetration, size_t _index)
		: pA(_pA), pB(_pB), point(_pt), penetration(_penetration), normal(_normal), index(_index)
	{
		//	ptA = pA->TransformPoint(localPtA * 0.8f);
		//	ptB = pB->TransformPoint(localPtB * 0.8f);

		//penetration = (ptA - ptB) | normal;
	}

	bool operator==(const SContactInfo& rhs) const
	{

		return (index == rhs.index) && ((pA == rhs.pA && pB == rhs.pB) || (pA == rhs.pB && pB == rhs.pA));
	}

	CPolygon* pA, *pB;

	Vec2	point;
	Vec2	normal;
	float	penetration;

	Vec2	edgeNormalA;
	Vec2	edgeNormalB;

	size_t	index;
};

struct SContact
{
	SContact() = default;
	SContact(const Vec2& _pt, const Vec2& _rA, const Vec2& _rB, const Vec2& _normal, float _penetration)
		: point(_pt), rA(_rA), rB(_rB), normal(_normal), penetration(_penetration), normalImpulse(0.0f), tangentImpulse(0.0f), normalVelocityBias(0.0f){}

	Vec2	point;
	Vec2	rA;
	Vec2	rB;
	Vec2	normal;
	float	penetration;

	float	normalImpulse;
	float	tangentImpulse;

	Vec2	edgeNormalA;
	Vec2	edgeNormalB;


	float	normalVelocityBias;
};


struct SCollision
{
	SCollision() = default;
	SCollision(CPolygonPtr _polyA, CPolygonPtr _polyB, Vec2	_point, Vec2 _normal, float _distance)
		: polyA(_polyA), polyB(_polyB), point(_point), normal(_normal), distance(_distance){}

	CPolygonPtr	polyA, polyB;

	Vec2	point;
	Vec2	normal;
	Vec2	normalDerivative;
	float	distance;



	size_t			manifoldSize = 0;
	SContactInfo	manifold[2];
};


#endif