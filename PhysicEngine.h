#ifndef _PHYSIC_ENGINE_H_
#define _PHYSIC_ENGINE_H_

#include <vector>
#include <unordered_map>
#include "Maths.h"
#include "Polygon.h"
#include "Collision.h"
#include "ContactConstraint.h"

class IBroadPhase;

class CPhysicEngine
{
public:
	// params
	float	restVelocityThreshold = 0.5f;
	float	restitution = 0.5f;
	float	staticFriction = 0.5f;
	float	slop = 0.01f;
	float	positionDampening = 0.5f;
	size_t	velocityIterations = 100;
	size_t	positionIterations = 5;
	float	rotationCoeff = 1.0f;


public:
	void	Reset();
	void	Activate(bool active);

	void	DetectCollisions();

	void	Step(float deltaTime);

	template<typename TFunctor>
	void	ForEachCollision(TFunctor functor)
	{
		for (const SCollision& collision : m_collidingPairs)
		{
			functor(collision);
		}
	}

private:
	void							CollisionBroadPhase();
	void							CollisionNarrowPhase();

	bool							m_active = true;

	// Collision detection
	IBroadPhase*					m_broadPhase;
	std::vector<SPolygonPair>		m_pairsToCheck;
	std::vector<SCollision>			m_collidingPairs;

	// Collision response
	std::vector<CContactConstraint>	m_contacts;
};

#endif