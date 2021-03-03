#include "PhysicEngine.h"

#include <iostream>
#include <string>
#include "GlobalVariables.h"
#include "World.h"
#include "Renderer.h" // for debugging only
#include "Timer.h"

#include "BroadPhase.h"
#include "BroadPhaseBrut.h"
#include "BroadPhaseSweepAndPrune.h"



void	CPhysicEngine::Reset()
{
	m_pairsToCheck.clear();
	m_collidingPairs.clear();

	m_active = true;

	m_broadPhase = new CBroadPhaseSweepAndPrune();
}

void	CPhysicEngine::Activate(bool active)
{
	m_active = active;
}

void	CPhysicEngine::DetectCollisions()
{
	CTimer timer;
	timer.Start();
	CollisionBroadPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision broadphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms");
	}

	timer.Start();
	CollisionNarrowPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision narrowphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms, collisions : " + std::to_string(m_collidingPairs.size()));
	}
}


void	CPhysicEngine::Step(float deltaTime)
{
	deltaTime = Min(deltaTime, 1.0f / 15.0f);

	if (!m_active)
	{
		return;
	}

	Vec2 gravity(0, -9.8f);
	float elasticity = 0.6f;

	gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
	{
		if (poly->density == 0.0f)
		{
			return;
		}

		poly->rotation.Rotate(RAD2DEG(poly->angularVelocity * deltaTime));
		poly->position += poly->speed * deltaTime;
		poly->speed += gravity * deltaTime;
	});

	DetectCollisions();

	m_contacts.clear();
	for (SCollision& collision : m_collidingPairs)
	{
		CContactConstraint contact(collision, rotationCoeff);
		contact.InitVelocityConstraint(deltaTime, restVelocityThreshold, restitution);
		m_contacts.push_back(contact);
	}

	for (size_t iteration = 0; iteration < velocityIterations; ++iteration)
	{
		for (CContactConstraint& contactConstraint : m_contacts)
		{
			contactConstraint.SolveVelocityConstraint(staticFriction);
		}
	}

	for (size_t iteration = 0; iteration < positionIterations; ++iteration)
	{
		for (CContactConstraint& contactConstraint : m_contacts)
		{
			contactConstraint.SolvePositionConstraint(slop, positionDampening, positionIterations);
		}
	}
}

void	CPhysicEngine::CollisionBroadPhase()
{
	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);
}

void	CPhysicEngine::CollisionNarrowPhase()
{
	m_collidingPairs.clear();

	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		SCollision collision;
		collision.polyA = pair.polyA;
		collision.polyB = pair.polyB;

		if (pair.polyA->CheckCollision(*(pair.polyB), collision))
		{
			m_collidingPairs.push_back(collision);
		}
	}
}