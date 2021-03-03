#ifndef __FLUID_SYSTEM_H__
#define __FLUID_SYSTEM_H__

#include "FluidMesh.h"
#include "Maths.h"
#include "Renderer.h"
#include "GlobalVariables.h"

#include <vector>
#include <string>

struct SParticleContact
{
	size_t	a, b;
	float	length;
};

class CFluidSystem
{
public:
	CFluidSystem()
	{
		float particuleRadius = m_radius / m_particleRadiusRatio;
		float volume = particuleRadius * particuleRadius * (float)M_PI;
		m_mass = volume * m_restDensity;
	}

	void	SetBounds(const Vec2& min, const Vec2& max)
	{
		m_min = min;
		m_max = max;
	}

	void	SpawnParticule(const Vec2& pos, const Vec2& vel)
	{
		m_positions.push_back(pos);
		m_velocities.push_back(vel);
		m_accImpulses.push_back(0.0f);
		m_accelerations.push_back(Vec2());
		m_densities.push_back(0.0f);
		m_pressures.push_back(0.0f);
	}

	void	Spawn(const Vec2& min, const Vec2& max, float particulesPerMeter, const Vec2& speed)
	{
		float width = max.x - min.x;
		size_t horiCount = (size_t)(width * particulesPerMeter) + 1;

		float height = max.y - min.y;
		size_t vertiCount = (size_t)(height * particulesPerMeter) + 1;

		size_t count = horiCount * vertiCount;

		m_positions.reserve(m_positions.size() + count);
		m_velocities.reserve(m_velocities.size() + count);
		m_accelerations.reserve(m_accelerations.size() + count);
		m_densities.reserve(m_densities.size() + count);
		m_pressures.reserve(m_pressures.size() + count);

		for (size_t i = 0; i < horiCount; ++i)
		{
			for (size_t j = 0; j < vertiCount; ++j)
			{
				float x = min.x + ((float)i) / particulesPerMeter;
				float y = min.y + ((float)j) / particulesPerMeter;

				SpawnParticule(Vec2(x, y), speed);
			}
		}
	}

	float _dt;

	void Update(float dt)
	{


		dt *= 0.5f;
		dt = Min(dt, 1.0f / 100.0f);

		_dt = dt;

		ResetAccelerations();

		FindContacts();
		ComputeDensity();
		ComputePressure();
		//AddPressureForces();
		//AddViscosityForces();


		ApplyForces(dt);

		for (float& impulse : m_accImpulses)
		{
			impulse = 0.0f;
		}

		float minDensity = (KernelDefault(0.0f, m_radius) * m_mass);
		float restDensity = (KernelDefault(0.0f, m_radius) * m_mass) * 1.0f;

		for (size_t k = 0; k < 25; ++k)
			for (size_t i = 0; i < m_positions.size(); ++i)
			{
				float rate = DensityRate(i);
				if (/*m_densities[i] > m_restDensity &&*/ rate > 0.0f || m_densities[i] > restDensity)// m_restDensity)
				{
					//float coeff = Clamp((m_densities[i] - minDensity) / (restDensity - minDensity), 0.0f, 1.0f);
					RelaxDensity(i, Max(rate, 0.0f) + Max(m_densities[i] - restDensity, 0.0f) * 0.0f);
				}
				BorderCollisions();
			}


		float v = 100.0f;
		float move = v * dt * dt;
		float h = m_radius;

		float minRad = h * 0.01f;
		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			if (m_densities[i] > restDensity)
			{
				Vec2 dx;
				Vec2& iPos = m_positions[i];
				std::vector<size_t>& neighbors = m_neighbors[i];
				for (size_t& j : neighbors)
				{
					Vec2& jPos = m_positions[j];

					Vec2 rVec = jPos - iPos;
					float r = Max(rVec.GetLength(), minRad);
					Vec2 rij = rVec / r;

					jPos += rij * (move * (m_densities[i] - restDensity) * KernelDefault(r, h));
					dx -= rij  * (move * (m_densities[i] - restDensity) * KernelDefault(r, h));
				}
				iPos += dx;
			}
		}


		ResetAccelerations();
		//AddViscosityForces();
		ClampArray(m_accelerations, 800.0f);
		for (size_t i = 0; i < m_velocities.size(); ++i)
		{
			//m_velocities[i] += m_accelerations[i] * dt;
		}

		Integrate(dt);


		FillMesh();
		m_mesh.Draw();
	}

private:
	void	ResetAccelerations()
	{
		for (Vec2& acc : m_accelerations)
		{
			acc = Vec2();
		}
	}

	void	FindContacts()
	{
		//m_neighbors.reserve(m_positions.size());
		m_neighbors.clear();
		for (size_t i = 0; i < m_positions.size(); ++i) m_neighbors.push_back(std::vector<size_t>());
		

		m_contacts.clear();

		float r = m_radius;
		float r2 = r * r;

		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			for (size_t j = i + 1; j < m_positions.size(); ++j)
			{
				const Vec2& iPos = m_positions[i];
				const Vec2& jPos = m_positions[j];

				float sqrLength = (iPos - jPos).GetSqrLength();

				if (sqrLength <= r2)
				{
					float d = (iPos - jPos).GetLength();
					SParticleContact contact;
					contact.a = i;
					contact.b = j;
					contact.length = d;

					m_neighbors[i].push_back(j);
					m_neighbors[j].push_back(i);

					m_contacts.push_back(contact);
				}
			}
		}
	}

	float	DensityRate(size_t i)
	{
		float h = m_radius;

		float minRad = h * 0.01f;

		float densityRate = 0.0f;
		Vec2& iPos = m_positions[i];

		std::vector<size_t>& neighbors = m_neighbors[i];
		for (size_t& j : neighbors)
		{
			Vec2& jPos = m_positions[j];

			Vec2 rVec = iPos - jPos;
			float r = Max(rVec.GetLength(), minRad);

			densityRate += (rVec | (m_velocities[i] - m_velocities[j])) * KernelSpikyGradientFactor(r, h);
		}
		densityRate *= m_mass;
		return densityRate;
	}

	void	RelaxDensity(size_t i, float densityRate)
	{
		float maxSpeed = m_radius / _dt;

		float h = m_radius;
		float minRad = h * 0.01f;

		Vec2& iPos = m_positions[i];

		Vec2 sumR;

		std::vector<size_t>& neighbors = m_neighbors[i];
		for (size_t& j : neighbors) 
		{
			Vec2& jPos = m_positions[j];
			float r = Max((jPos - iPos).GetLength(), minRad);

			Vec2 rij = (jPos - iPos) / r;
			sumR += rij;
		}
		sumR /= m_densities[i];

		float denom = 0.0f;
		for (size_t& j : neighbors)
		{
			Vec2& jPos = m_positions[j];

			Vec2 rVec = iPos - jPos;
			float r = Max(rVec.GetLength(), minRad);

			Vec2 rij = rVec / -r;

			denom += (rVec | (sumR + rij / m_densities[j])) * KernelSpikyGradientFactor(r, h);
		}
		denom *= -m_mass;
		if (fabsf(denom) < 1e-5f)
		{
			return;
		}

		float impulse = -densityRate / denom;

		float accImpulse = m_accImpulses[i];

		
		impulse = Max(impulse, -accImpulse);
		impulse = Min(impulse, 0.05f - accImpulse);
		if (impulse <= 0.0f)
		{
			return;
		}

		m_accImpulses[i] += impulse;

		m_velocities[i] += sumR * -impulse;


		for (size_t& j : neighbors)
		{
			Vec2& jPos = m_positions[j];

			Vec2 rVec = iPos - jPos;
			float r = Max(rVec.GetLength(), minRad);
			Vec2 rij = rVec / -r;

			m_velocities[j] += rij * (impulse / m_densities[j]);
		}

		float newRate = DensityRate(i);


		return;
	}


	void	ComputeDensity()
	{
		float radius = m_radius;
		float mass = m_mass;

		float baseWeight = KernelDefault(0.0f, radius);

		for (float& density : m_densities)
		{
			density = baseWeight;
		}

		for (SParticleContact& contact : m_contacts)
		{
			const Vec2& aPos = m_positions[contact.a];
			const Vec2& bPos = m_positions[contact.b];

			float weight = KernelDefault(contact.length, radius);
			m_densities[contact.a] += weight;
			m_densities[contact.b] += weight;
		}

		for (float& density : m_densities)
		{
			density *= mass;
		}
	}

	void	ComputePressure()
	{
		for (size_t i = 0; i < m_pressures.size(); ++i)
		{
			m_pressures[i] = Max(m_stiffness * (m_densities[i] - m_restDensity), 0.0f);
		}
	}

	void	AddPressureForces()
	{
		float radius = m_radius;
		float mass = m_mass;

		for (SParticleContact& contact : m_contacts)
		{
			const Vec2& aPos = m_positions[contact.a];
			const Vec2& bPos = m_positions[contact.b];

			Vec2 r = aPos - bPos;
			float length = Max(contact.length, 1e-12f);

			Vec2 pressureAcc = r * -mass * ((m_pressures[contact.a] + m_pressures[contact.b]) / (2.0f * m_densities[contact.a] * m_densities[contact.b])) * KernelSpikyGradientFactor(length, radius);
			m_accelerations[contact.a] += pressureAcc;
			m_accelerations[contact.b] -= pressureAcc;
		}
	}

	void	AddViscosityForces()
	{
		float radius = m_radius;
		float mass = m_mass;
		float viscosity = m_viscosity;

		for (SParticleContact& contact : m_contacts)
		{
			const Vec2& aPos = m_positions[contact.a];
			const Vec2& bPos = m_positions[contact.b];

			Vec2 deltaVel = m_velocities[contact.a] - m_velocities[contact.b];
			Vec2 viscosityAcc = deltaVel * -mass * (viscosity / (2.0f * m_densities[contact.a] * m_densities[contact.b])) * KernelViscosityLaplacian(contact.length, radius);
		
			m_accelerations[contact.a] += viscosityAcc;
			m_accelerations[contact.b] -= viscosityAcc;
		}
	}


	void	BorderCollisions()
	{
		const float restitution = 0.1f; // 0.1;// 0.6f;

		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			Vec2& pos = m_positions[i];
			if (pos.x < m_min.x && m_velocities[i].x < 0.0f)
			{
				pos.x = m_min.x;
				m_velocities[i].x *= -restitution;
			}
			else if (pos.x > m_max.x && m_velocities[i].x > 0.0f)
			{
				pos.x = m_max.x;
				m_velocities[i].x *= -restitution;
			}

			if (pos.y < m_min.y  && m_velocities[i].y < 0.0f)
			{
				pos.y = m_min.y;
				m_velocities[i].y *= -restitution;
			}
			else if (pos.y > m_max.y  && m_velocities[i].y > 0.0f)
			{
				pos.y = m_max.y;
				m_velocities[i].y *= -restitution;
			}
		}
	}

	void	ApplyForces(float dt)
	{
		ClampArray(m_accelerations, 800.0f);

		Vec2 gravity(0.0f, -9.8f);
		for (size_t i = 0; i < m_velocities.size(); ++i)
		{
			m_velocities[i] += (m_accelerations[i] + gravity) * dt;
		}
	}

	void	Integrate(float dt)
	{
		ClampArray(m_velocities, 10.0f);
		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			m_positions[i] += m_velocities[i] * dt;
		}
	}

	void	ClampArray(std::vector<Vec2>& array, float limit)
	{
		for (Vec2& vec : array)
		{
			if (vec.GetSqrLength() > limit * limit)
			{
				vec *= limit / vec.GetLength();
			}
		}
	}

	void	FillMesh()
	{
		m_mesh.Fill(m_positions.size(), [&](size_t iVertex, float& x, float& y, float& r, float& g, float& b)
		{
			const Vec2& pos = m_positions[iVertex];
			x = pos.x;
			y = pos.y;
			r = 1.0f;
			g = 0.0f;
			b = 0.0f;
		});
	}

	float				m_radius = 0.2f;
	float				m_restDensity = 0.59f;
	float				m_stiffness = 1000.0f;
	float				m_particleRadiusRatio = 3.0f;
	float				m_viscosity = 1;// 0.1f;

	float				m_mass;

	std::vector<Vec2>	m_positions;
	std::vector<Vec2>	m_accelerations;
	std::vector<Vec2>	m_velocities;
	std::vector<float>	m_accImpulses;
	std::vector<float>	m_densities;
	std::vector<float>	m_pressures;

	std::vector< std::vector<size_t> >	m_neighbors;

	std::vector<SParticleContact>	m_contacts;

	Vec2		m_min, m_max;
	CFluidMesh	m_mesh;
};

#endif