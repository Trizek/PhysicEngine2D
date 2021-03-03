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

struct SParticleProxy
{
	SParticleProxy(size_t _i) : i(_i){}

	size_t i;
};

class CFluidSystem
{
public:
	CFluidSystem()
	{
		float particuleRadius = m_radius / m_particleRadiusRatio;
		float volume = particuleRadius * particuleRadius * (float)M_PI;
		m_mass = volume * m_restDensity;
		m_minRadius = m_radius * 0.1f;
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
		m_accelerations.push_back(Vec2());
		m_densities.push_back(0.0f);
		m_pressures.push_back(0.0f);
		m_keys.push_back(0);
		m_proxies.push_back(SParticleProxy(m_proxies.size()));
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


	std::vector<Vec2> m_prevPos;

	void Update(float dt)
	{
		//dt *= 0.5f;
		dt = Min(dt, 1.0f / 200.0f);

		_dt = dt;

		float dt2 = dt * dt;

		float radius = m_radius;
		float mass = m_mass;
		float minRadius = m_minRadius;

		m_prevPos.clear();

		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			m_velocities[i].y -= 9.8f * dt;
		}

		float viscosity = m_viscosity;

		FindContacts();
		ComputeDensity();

		for (SParticleContact& contact : m_contacts)
		{
			Vec2& aPos = m_positions[contact.a];
			Vec2& bPos = m_positions[contact.b];

			Vec2 r = aPos - bPos;
			float length = contact.length;

			Vec2 deltaVel = m_velocities[contact.a] - m_velocities[contact.b];
			Vec2 viscosityAcc = deltaVel * -mass * (viscosity / (2.0f * m_densities[contact.a] * m_densities[contact.b])) * KernelViscosityLaplacian(length, radius);

			m_velocities[contact.a] += viscosityAcc * dt;
			m_velocities[contact.b] -= viscosityAcc * dt;
		}

		ClampArray(m_velocities, 10.0f);

		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			m_prevPos.push_back(m_positions[i]);
			m_positions[i] += m_velocities[i] * dt;
		}


		ResetAccelerations();

	UpdateContacts();
		//FindContacts();
		ComputeDensity();
		ComputePressure();

		{



			
			for (SParticleContact& contact : m_contacts)
			{
				Vec2& aPos = m_positions[contact.a];
				Vec2& bPos = m_positions[contact.b];

				Vec2 r = aPos - bPos;
				float length = contact.length;



				float acc = -mass * ((m_pressures[contact.a] + m_pressures[contact.b]) / (2.0f * m_densities[contact.a] * m_densities[contact.b])) * KernelSpikyGradientFactor(length, radius);
				acc *= length;

				float limitAcc = 10.0f / dt;
				acc = Clamp(acc, -limitAcc, limitAcc);

				Vec2 pressureAcc = r * (acc/length);
				
		

				aPos += pressureAcc * dt2;
				bPos -= pressureAcc * dt2;
				//m_accelerations[contact.a] += pressureAcc;
				//m_accelerations[contact.b] -= pressureAcc;
			}
		}

		

		//AddPressureForces();
		//AddViscosityForces();


	//	Integrate(dt);


		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			m_velocities[i] = (m_positions[i] - m_prevPos[i]) / dt;
		}

		BorderCollisions();

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

	void	ComputeKeys()
	{
		float h = m_radius * 2;

		size_t shift = (sizeof(int) * 8) / 2;

		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			Vec2 pos = m_positions[i];
			int key = (int)floor(pos.y / h) + (((int)floor(pos.x / h)) << shift);
			m_keys[i] = key;
		}
	}

	int		GetRightKey(int key)
	{
		size_t shift = (sizeof(int) * 8) / 2;
		return key + (1 << shift);
	}

	int		GetTopKey(int key)
	{
		return key + 1;
	}


	void	UpdateProxies()
	{
		for (int a = m_proxies.size() - 2; a >= 0; --a)
		{
			// insert a in sorted [a + 1, last]
			SParticleProxy aProxy = m_proxies[a];
			size_t b = (size_t)a + 1;
			int aKey = m_keys[aProxy.i];
			while (b < m_proxies.size() && aKey > m_keys[m_proxies[b].i])
			{
				// swap a (which is in b-1 currently) and b proxies
				m_proxies[b - 1] = m_proxies[b];
				m_proxies[b] = aProxy;
				++b;
			}
		}
	}

	void	AddProxyContacts(size_t a)
	{
		float h = m_radius;

		int aKey = m_keys[m_proxies[a].i];
		int topKey = GetTopKey(aKey);

		// iterate over all particles in same cell and cell just to the top
		size_t b = a + 1;
		for (; b < m_proxies.size() && m_keys[m_proxies[b].i] <= topKey; ++b)
		{
			AddContact(m_proxies[a].i, m_proxies[b].i, h);
		}

		int rightKey = GetRightKey(aKey);
		for (; b < m_proxies.size() && m_keys[m_proxies[b].i] < rightKey; ++b);

		int topRightKey = GetTopKey(rightKey);
		for (; b < m_proxies.size() && m_keys[m_proxies[b].i] <= topRightKey; ++b)
		{
			AddContact(m_proxies[a].i, m_proxies[b].i, h);
		}
	}

	void	AddContact(size_t i, size_t j, float h)
	{
		const Vec2& iPos = m_positions[i];
		const Vec2& jPos = m_positions[j];

		float sqrLength = (iPos - jPos).GetSqrLength();

		if (sqrLength <= h * h)
		{
			float d = (iPos - jPos).GetLength();
			SParticleContact contact;
			contact.a = i;
			contact.b = j;
			contact.length = Clamp(d, m_minRadius, h);

			m_contacts.push_back(contact);
		}
	}

	void	FindContacts()
	{
		ComputeKeys();
		UpdateProxies();

		m_contacts.clear();

		float h = m_radius;

		if (true)
		{
			for (size_t a = 0; a < m_proxies.size(); ++a)
			{
				AddProxyContacts(a);
			}

			//for (size_t i = 0; i < m_contacts.size(); ++i)
			//{
			//	SParticleContact& iC = m_contacts[i];
			//	for (size_t j = i + 1; j < m_contacts.size(); ++j)
			//	{
			//		SParticleContact& jC = m_contacts[j];
			//		if ((iC.a == jC.a && iC.b == jC.b) || (iC.a == jC.b && iC.b == jC.a))
			//		{
			//			exit(0);
			//		}
			//	}
			//}
		}
		else
		{			
			for (size_t i = 0; i < m_positions.size(); ++i)
			{
				for (size_t j = i + 1; j < m_positions.size(); ++j)
				{
					AddContact(i, j, h);
				}
			}
		}
	}

	void	UpdateContacts()
	{
		float h = m_radius;

		for (SParticleContact& contact : m_contacts)
		{
			const Vec2& aPos = m_positions[contact.a];
			const Vec2& bPos = m_positions[contact.b];
			contact.length = Clamp((aPos - bPos).GetLength(), m_minRadius, h);
		}
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
		float minDensity = (KernelDefault(0.0f, m_radius) * m_mass);
		for (size_t i = 0; i < m_pressures.size(); ++i)
		{
			m_pressures[i] = m_stiffness * ((m_densities[i] - m_restDensity) + 0.3f * (m_densities[i] - minDensity));//Max(m_stiffness * ((m_densities[i] - m_restDensity) + 0.3f * (m_densities[i] - minDensity)), 0.0f);
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
		const float restitution = 0.4f;// 0.6f; // 0.1;// 0.6f;
		const float friction = 0.4f;

		for (size_t i = 0; i < m_positions.size(); ++i)
		{
			Vec2& pos = m_positions[i];
			if (pos.x <= m_min.x && m_velocities[i].x < 0.0f)
			{
				pos.x = m_min.x;
				m_velocities[i].x *= -restitution;
				m_velocities[i].y *= friction;
			}
			else if (pos.x >= m_max.x && m_velocities[i].x > 0.0f)
			{
				pos.x = m_max.x;
				m_velocities[i].x *= -restitution;
				m_velocities[i].y *= friction;
			}

			if (pos.y <= m_min.y  && m_velocities[i].y < 0.0f)
			{
				pos.y = m_min.y;
				m_velocities[i].y *= -restitution;
				m_velocities[i].x *= friction;
			}
			else if (pos.y >= m_max.y  && m_velocities[i].y > 0.0f)
			{
				pos.y = m_max.y;
				m_velocities[i].y *= -restitution;
				m_velocities[i].x *= friction;
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
			r = 0.0f;
			g = 0.0f;
			b = 1.0f;
		});
	}

	float				m_radius = 0.2f;
	float				m_minRadius;
	float				m_restDensity = 0.59f;
	float				m_stiffness = 500.0f; // 1000.0f;
	float				m_particleRadiusRatio = 3.0f;
	float				m_viscosity = 0.1f;// 0.1f;// 0.1f;// 0.1f;// 0.1f;

	float				m_mass;

	std::vector<Vec2>	m_positions;
	std::vector<Vec2>	m_accelerations;
	std::vector<Vec2>	m_velocities;
	std::vector<float>	m_densities;
	std::vector<float>	m_pressures;
	std::vector<int>	m_keys;
	std::vector<SParticleProxy>	m_proxies;


	std::vector<SParticleContact>	m_contacts;

	Vec2		m_min, m_max;
	CFluidMesh	m_mesh;
};

#endif