#include "ContactConstraint.h"

#include "Renderer.h"
#include "GlobalVariables.h"

CContactConstraint::CContactConstraint(SCollision& collision, float rotationCoeff)
	: m_pA(collision.polyA)
	, m_pB(collision.polyB)
	, m_manifoldSize(collision.manifoldSize)
	, m_redundant(false)
{
	m_invMassA = (m_pA->density > 0.0f) ? 1.0f / m_pA->GetMass() : 0.0f;
	m_invMassB = (m_pB->density > 0.0f) ? 1.0f / m_pB->GetMass() : 0.0f;
	m_invTensorA = ((m_pA->density > 0.0f) ? 1.0f / m_pA->GetInertiaTensor() : 0.0f) * rotationCoeff;
	m_invTensorB = ((m_pB->density > 0.0f) ? 1.0f / m_pB->GetInertiaTensor() : 0.0f) * rotationCoeff;


	for (size_t i = 0; i < collision.manifoldSize; ++i)
	{
		m_manifold[i] = SContact(collision.manifold[i].point, collision.manifold[i].point - m_pA->position, collision.manifold[i].point - m_pB->position, collision.manifold[i].normal, collision.manifold[i].penetration);
		m_manifold[i].edgeNormalA = collision.manifold[i].edgeNormalA;
		m_manifold[i].edgeNormalB = collision.manifold[i].edgeNormalB;
	}
}

void CContactConstraint::InitVelocityConstraint(float deltaTime, float restVelocityThreshold, float restitution)
{
	for (size_t i = 0; i < m_manifoldSize; ++i)
	{
		float vn = (m_pB->GetPointVelocity(m_manifold[i].point) - m_pA->GetPointVelocity(m_manifold[i].point)) | m_manifold[i].normal;
		m_manifold[i].normalVelocityBias = Select(vn < -restVelocityThreshold, -vn * restitution, 0.0f);
	}

	if (m_manifoldSize == 2)
	{
		float rA1CrossN = m_manifold[0].rA ^ m_manifold[0].normal;
		float rB1CrossN = m_manifold[0].rB ^ m_manifold[0].normal;
		float rA2CrossN = m_manifold[1].rA ^ m_manifold[1].normal;
		float rB2CrossN = m_manifold[1].rB ^ m_manifold[1].normal;

		float a11 = m_invMassA + m_invMassB + rA1CrossN * rA1CrossN * m_invTensorA + rB1CrossN * rB1CrossN * m_invTensorB;
		float a22 = m_invMassA + m_invMassB + rA2CrossN * rA2CrossN * m_invTensorA + rB2CrossN * rB2CrossN * m_invTensorB;
		float a12 = m_invMassA + m_invMassB + rA1CrossN * rA2CrossN * m_invTensorA + rB1CrossN * rB2CrossN * m_invTensorB;

		m_JWJT.X = Vec2(a11, a12);
		m_JWJT.Y = Vec2(a12, a22);

		if (fabsf(m_JWJT.GetDeterminant()) < 1e-5f)
		{
			m_redundant = true;
		}
		else
		{
			m_JWJTInverse = m_JWJT.GetInverse();
		}
	}
}

void CContactConstraint::SolveVelocityConstraint(float staticFriction)
{
	Vec2&	vA = m_pA->speed;
	Vec2&	vB = m_pB->speed;
	float&	wA = m_pA->angularVelocity;
	float&	wB = m_pB->angularVelocity;

	bool patchSolveSucceed = false;

	// solve friction first
	for (size_t i = 0; i < m_manifoldSize; ++i)
	{
		float& normalImpulse = m_manifold[i].normalImpulse;
		float& tangentImpulse = m_manifold[i].tangentImpulse;

		Vec2 t = m_manifold[i].normal.GetNormal();;
		float rACrossT = m_manifold[i].rA ^ t;
		float rBCrossT = m_manifold[i].rB ^ t;

		float JV = -(t | vA) - rACrossT * wA + (t | vB) + rBCrossT * wB;
		float effectiveMass = m_invMassA + m_invMassB + rACrossT * rACrossT * m_invTensorA + rBCrossT * rBCrossT * m_invTensorB;

		float lambda = -JV / effectiveMass;

		lambda = Clamp(lambda, -staticFriction* normalImpulse - tangentImpulse, staticFriction * normalImpulse - tangentImpulse);
		m_manifold[i].tangentImpulse += lambda;

		vA -= t * m_invMassA * lambda;
		wA -= rACrossT * m_invTensorA * lambda;
		vB += t * m_invMassB * lambda;
		wB += rBCrossT * m_invTensorB * lambda;
	}

	// non-penetration
	if (m_manifoldSize == 2 && !m_redundant)
	{
		// 2 contacts manifold, we solve the LCP for those 2 contacts at the same time (analytic solution for max stability)

		// Solve J(V + deltaV) = J(V + WJT * d) = JV + JWJT * d >= velocityBias   (a : accumulated impulse, i : incremental impulse)
		// A = JWJT, x = a + d, b = -bias - JWJT * a - JV
		// Solve A*x + b >= 0 with transpose(x)*(A*x + b) = 0 (LCP) (either impulse (xi) = 0, either speed (A*x + b)i = 0, impulse separate only if needed)

		const Vec2& n = m_manifold[0].normal;
		float rA1CrossN = m_manifold[0].rA ^ n;
		float rB1CrossN = m_manifold[0].rB ^ n;
		float rA2CrossN = m_manifold[1].rA ^ n;
		float rB2CrossN = m_manifold[1].rB ^ n;

		Vec2 bias(m_manifold[0].normalVelocityBias, m_manifold[1].normalVelocityBias);
		Vec2 a(m_manifold[0].normalImpulse, m_manifold[1].normalImpulse);
		Vec2 JV(-(n | vA) - rA1CrossN * wA + (n | vB) + rB1CrossN * wB, -(n | vA) - rA2CrossN * wA + (n | vB) + rB2CrossN * wB);

		Vec2 b = JV - bias - m_JWJT * a;

		Vec2 accumulatedImpulses;
		if (Solve2DLCP(m_JWJT, m_JWJTInverse, b, accumulatedImpulses))
		{
			float& normalImpulse1 = m_manifold[0].normalImpulse;
			float& normalImpulse2 = m_manifold[1].normalImpulse;

			float lambda1 = accumulatedImpulses.x - normalImpulse1;
			float lambda2 = accumulatedImpulses.y - normalImpulse2;

			normalImpulse1 = accumulatedImpulses.x;
			normalImpulse2 = accumulatedImpulses.y;

			vA -= n * m_invMassA * (lambda1 + lambda2);
			wA -= (rA1CrossN * lambda1 + rA2CrossN * lambda2) * m_invTensorA;
			vB += n * m_invMassB * (lambda1 + lambda2);
			wB += (rB1CrossN * lambda1 + rB2CrossN * lambda2) * m_invTensorB;
			patchSolveSucceed = true;
		}
	}

	for (size_t i = 0; i < m_manifoldSize && !patchSolveSucceed; ++i)
	{
		float& normalImpulse = m_manifold[i].normalImpulse;

		const Vec2& n = m_manifold[i].normal;
		float rACrossN = m_manifold[i].rA ^ n;
		float rBCrossN = m_manifold[i].rB ^ n;

		float JV = -(n | vA) - rACrossN * wA + (n | vB) + rBCrossN * wB;
		float effectiveMass = m_invMassA + m_invMassB + rACrossN * rACrossN * m_invTensorA + rBCrossN * rBCrossN * m_invTensorB;

		float lambda = (m_manifold[i].normalVelocityBias - JV) / effectiveMass;
		lambda = Max(-normalImpulse, lambda);

		normalImpulse += lambda;

		vA -= n * m_invMassA * lambda;
		wA -= rACrossN * m_invTensorA * lambda;
		vB += n * m_invMassB * lambda;
		wB += rBCrossN * m_invTensorB * lambda;
	}
}

void CContactConstraint::SolvePositionConstraint(float slop, float dampening, size_t iterations)
{
	Vec2&	vA = m_pA->speed;
	Vec2&	vB = m_pB->speed;
	float&	wA = m_pA->angularVelocity;
	float&	wB = m_pB->angularVelocity;

	for (size_t i = 0; i < m_manifoldSize; ++i)
	{
		float dist = Max(m_manifold[i].penetration - slop, 0.0f);
		if (dist <= 0.0f)
			continue;

		const Vec2& n = m_manifold[i].normal;
		float rACrossN = m_manifold[i].rA ^ n;
		float rBCrossN = m_manifold[i].rB ^ n;

		float factor = 1.0f / (float)m_manifoldSize;
		dist *= factor * (dampening /(float)iterations);
		dist /= m_invMassA + m_invMassB + rACrossN * rACrossN * m_invTensorA * 0 + rBCrossN * rBCrossN * m_invTensorB * 0;

		Vec2 separation = m_manifold[i].normal * dist;

		m_pA->position -= separation * m_invMassA;
		m_pA->rotation.Rotate(RAD2DEG(-rACrossN * m_invTensorA * 0 * dist));
		m_pB->position += separation * m_invMassB;
		m_pB->rotation.Rotate(RAD2DEG(rBCrossN * m_invTensorB * 0 * dist));
	}
}

void CContactConstraint::DebugDraw() const
{
	for (size_t i = 0; i < m_manifoldSize; ++i)
	{
		gVars->pRenderer->DrawLine(m_manifold[i].point, m_pA->position, 0, 1, 0);
		gVars->pRenderer->DrawLine(m_manifold[i].point, m_pB->position, 0, 0, 1);
	}
}