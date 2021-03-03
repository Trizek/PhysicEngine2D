#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <GL/glew.h>
#include <vector>
#include <memory>


#include "Maths.h"


struct SFeature
{
	SFeature() = default;
	SFeature(bool _bPoint, size_t _index)
		: bPoint(_bPoint), index(_index){}

	bool	bPoint;
	size_t	index;
};

struct SCacheContact
{
	size_t	otherPoly;
	Vec2	r;
	float	normalImpulse;
	float	tangentImpulse;
	int		life;
};

struct SCacheManifold
{
	size_t size;
	SCacheContact contacts[8];
};

class CPolygon
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;
	std::vector<Vec2>	points;
	AABB				aabb;

	void				Build();
	void				Draw();
	size_t				GetIndex() const;

	float				GetArea() const;

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	bool				CheckCollision2(CPolygon& poly, struct SCollision& collision);


	// If line intersect polygon, colDist is the penetration distance, and colPoint most penetrating point of poly inside the line
	bool				IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const;
	bool				CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const;

	float				GetInvSupport(const Vec2& point, const Vec2& dir, SFeature& feature);
	
//	bool				CheckCollision(CPolygon& poly, struct SCollision& collision);

	float				GetSupport(const Vec2& point, const Vec2& dir) const;
	float				GetInvSupport(const Vec2& point, const Vec2& dir) const;
	float				GetMaxSeparationEdge(size_t &edgeIndex, const CPolygon& poly) const;
	size_t				GetOpposingEdge(const Vec2& normal) const;
	bool				CheckCollision(CPolygon& poly, struct SCollision& collision);

	float				UnProjectPoint(const Vec2& point, const Vec2& dir, float dist);
	float				UnProject(CPolygon& poly, const Vec2& dir, float dist);





	void				UpdateAABB();

	float				GetMass() const;
	float				GetInertiaTensor() const;

	Vec2				GetPointVelocity(const Vec2& point) const;

	// Physics
	float				density;
	Vec2				speed;
	float				angularVelocity = 0.0f;
	Vec2				forces;
	float				torques = 0.0f;

	SCacheManifold		cacheManifold;

	bool				GetCacheContact(CPolygon* otherPoly, const Vec2& rA, const Vec2& rB, float& normalImpulse, float& tangentImpulse)
	{
		if (m_index < otherPoly->m_index)
		{
			bool res = otherPoly->GetCacheContact(this, rB, rA, normalImpulse, tangentImpulse);
			normalImpulse *= -1.0f;
			tangentImpulse *= -1.0f;
			return res;
		}

		size_t size = cacheManifold.size;
		for (size_t i = 0; i < size; ++i)
		{
			SCacheContact& contact = cacheManifold.contacts[i];
			if (contact.otherPoly == otherPoly->m_index && (contact.r - rA).GetSqrLength() < 0.1f * 0.1f)
			{
				normalImpulse = contact.normalImpulse;
				tangentImpulse = contact.tangentImpulse;
				return true;
			}
		}
		
		return false;
	}

	void				CacheContact(CPolygon* otherPoly, const Vec2& rA, const Vec2& rB, float normalImpulse, float tangentImpulse)
	{
		if (m_index < otherPoly->m_index)
		{
			otherPoly->CacheContact(this, rB, rA, -normalImpulse, -tangentImpulse);
			return;
		}

		SCacheContact* pContact = nullptr;

		size_t& size = cacheManifold.size;
		for (size_t i = 0; i < size; ++i)
		{
			SCacheContact& contact = cacheManifold.contacts[i];
			if (contact.otherPoly == otherPoly->m_index && (contact.r - rA).GetSqrLength() < 0.001f * 0.001f)
			{
				pContact = &contact;
				break;
			}
		}

		if (pContact == nullptr)
		{
			if (size < 8)
			{
				pContact = &(cacheManifold.contacts[0]) + size++;
				pContact->otherPoly = otherPoly->m_index;
				pContact->r = rA;
			}
			else
			{
				return;
			}
		}

		pContact->life = 1;
		pContact->normalImpulse = normalImpulse;
		pContact->tangentImpulse = tangentImpulse;
	}

	void				UpdateCacheManifold()
	{
		int size = cacheManifold.size;
		for (int i = size - 1; i >= 0; --i)
		{
			SCacheContact& contact = cacheManifold.contacts[i];
 			if (--contact.life < 0)
			{
				contact = cacheManifold.contacts[size - 1];
				cacheManifold.size--;
			}
		}
	}


private:
	void				CreateBuffers();
	void				BindBuffers();
	void				DestroyBuffers();

	void				BuildLines();

	void				ComputeArea();
	void				RecenterOnCenterOfMass(); // Area must be computed
	void				ComputeLocalInertiaTensor(); // Must be centered on center of mass

	GLuint				m_vertexBufferId;
	size_t				m_index;

	std::vector<Line>	m_lines;

	float				m_signedArea;

	// Physics
	float				m_localInertiaTensor; // don't consider mass
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif