#ifndef _CONTACT_CONSTRAINT_H_
#define _CONTACT_CONSTRAINT_H_

#include "Maths.h"

#include "Collision.h"

struct CContactConstraint
{
public:
	CContactConstraint(SCollision& collision, float rotationCoeff);

	void		InitVelocityConstraint(float deltaTime, float restVelocityThreshold, float restitution);
	void		SolveVelocityConstraint(float staticFriction);

	void		SolvePositionConstraint(float slop, float dampening, size_t iterations);

	void		DebugDraw() const;

private:
	CPolygonPtr m_pA, m_pB;

	float		m_invMassA;
	float		m_invMassB;
	float		m_invTensorA;
	float		m_invTensorB;

	size_t		m_manifoldSize;
	SContact	m_manifold[2];

	Mat2		m_JWJT;
	Mat2		m_JWJTInverse; // effective mass

	bool		m_redundant;
};

#endif