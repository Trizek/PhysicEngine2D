#ifndef _DISPLAY_MANIFOLD_H_
#define _DISPLAY_MANIFOLD_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "World.h"

#include <string>
#include <iostream>

class CDisplayManifold : public CBehavior
{
public:
	CPolygonPtr polyA;
	CPolygonPtr polyB;

private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->Activate(false);

		//DrawCollisionPolygon(polyA);
		//DrawCollisionPolygon(polyB);

		gVars->pRenderer->DisplayTextWorld("A", polyA->position);
		gVars->pRenderer->DisplayTextWorld("B", polyB->position);

		Vec2 dir = Vec2(-1.0f, -0.5f).Normalized();
		float dist = 100.0f;

		float d = Min(polyA->UnProject(*polyB, dir, dist), dist);
		if (d >= 0.0f)
		{
			DrawGhostPolygon(polyA, dir * d);
		}




		SCollision collision;
		if (false) //polyA->CheckCollision(*polyB, collision))
		{
			Vec2 normal = collision.manifold[0].normal;
			float penetration = 0.0f;

			for (size_t i = 0; i < collision.manifoldSize; ++i)
			{
				gVars->pRenderer->DrawLine(collision.manifold[i].point, collision.manifold[i].point + collision.manifold[i].normal * collision.manifold[i].penetration, 0, 0, 1);
			}

		//	Vec2 offset = normal * penetration;
			//polyB->position += normal.Normalized() * Select(penetration > 0, 1, 0) *1.0f * frameTime;
		}
	}

	void DrawCollisionPolygon(CPolygonPtr poly)
	{
		for (size_t i = 0; i < poly->points.size(); ++i)
		{
			Vec2 pointA = poly->TransformPoint(poly->points[i] * 0.6f);
			Vec2 pointB = poly->TransformPoint(poly->points[(i + 1) % poly->points.size()] * 0.6f);

			gVars->pRenderer->DrawLine(pointA, pointB, 0, 1, 0);
		}
	}

	void DrawGhostPolygon(CPolygonPtr poly, Vec2 offset)
	{
		for (size_t i = 0; i < poly->points.size(); ++i)
		{
			Vec2 pointA = poly->TransformPoint(poly->points[i]) + offset;
			Vec2 pointB = poly->TransformPoint(poly->points[(i + 1) % poly->points.size()]) + offset;

			gVars->pRenderer->DrawLine(pointA, pointB, 0, 1, 0);
		}
	}
};


#endif