#ifndef _FLUID_SIMULATION_H_
#define _FLUID_SIMULATION_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

#define RADIUS 0.9f //2.0f
#define DISTANCE 5.0f

struct SCircle
{
	Vec2 pos, speed;
};


class CFluidSimulation: public CBehavior
{
private:


	virtual void Start() override
	{
		for (float x = -12.0f; x < 12.0f; x += 1.0f)
		{
			for (float y = -22.0f; y < 22.0f; y += 1.0f)
			{
				AddCircle(Vec2(x + Random(-0.1f, 0.1f), y + Random(-0.1f, 0.1f)));
			}
		}

		gVars->pPhysicEngine->Activate(false);
	}

	virtual void Update(float frameTime) override
	{
		for (SCircle& circle : m_circles)
		{
			circle.speed.y -= 20.0f * frameTime;
			circle.speed -= circle.speed * 0.3f * frameTime;
		}

		for (size_t i = 0; i < m_circles.size(); ++i)
		{
			for (size_t j = i + 1; j < m_circles.size(); ++j)
			{
				SCircle& c1 = m_circles[i];
				SCircle& c2 = m_circles[j];

				Vec2 diffPos = c2.pos - c1.pos;
				Vec2 diffSpeed = c2.speed - c1.speed;
				if (diffPos.GetSqrLength() < 4.0f * RADIUS * RADIUS && ((diffSpeed | diffPos) < 0.0f))
				{
					Vec2 normal = diffPos.Normalized();
					Vec2 diff = normal * (diffSpeed | normal) * 0.5f;

					c1.speed += diff * 1.4f;
					c2.speed -= diff * 1.4f;
					//float colDist = 2.0f * RADIUS - diffPos.GetLength();

					//c1->position -= normal * colDist * 0.5f;
					//c2->position += normal * colDist * 0.5f;
				}
			}
		}




		float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
		float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

		for (SCircle& circle : m_circles)
		{
			if (circle.pos.x < -hWidth && circle.speed.x < 0)
			{
				circle.speed.x *= -1.0f;
			}
			else if (circle.pos.x > hWidth && circle.speed.x > 0)
			{
				circle.speed.x *= -1.0f;
			}
			if (circle.pos.y < -hHeight && circle.speed.y < 0)
			{
				circle.speed.y *= -1.0f;
			}
			else if (circle.pos.y > hHeight && circle.speed.y > 0)
			{
				circle.speed.y *= -1.0f;
			}
		}

		for (SCircle& circle : m_circles)
		{
			circle.pos += circle.speed * frameTime;
		}

		UpdatePolys();
	}

	void AddCircle(const Vec2& pos, float radius = RADIUS)
	{
		SCircle circle;
		circle.pos = pos;
		circle.speed.x = 50;
	


		CPolygonPtr poly = gVars->pWorld->AddSymetricPolygon(radius, 3); // 5);
		poly->density = 0.0f;
		poly->position = pos;
		poly->speed = circle.speed;

		m_poly.push_back(poly);
		m_circles.push_back(circle);

	}
	
	void UpdatePolys()
	{
		for (size_t i = 0; i < m_circles.size(); ++i)
		{
			CPolygonPtr poly = m_poly[i];
			SCircle& circle = m_circles[i];

			poly->position = circle.pos;
			poly->speed = circle.speed;
		}
	}


	std::vector<CPolygonPtr>	m_poly;
	std::vector<SCircle>	m_circles;
};

#endif