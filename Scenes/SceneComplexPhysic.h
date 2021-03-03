#ifndef _SCENE_COMPLEX_PHYSIC_H_
#define _SCENE_COMPLEX_PHYSIC_H_

#include "BaseScene.h"


class CSceneComplexPhysic : public CBaseScene
{
public:
	CSceneComplexPhysic(size_t polyCount, float scale = 2.0f) : CBaseScene(0.5f * scale, 10.0f * scale), m_scale(scale), m_polyCount(polyCount){}

private:
	virtual void Create() override
	{
		CBaseScene::Create();

		float width = gVars->pRenderer->GetWorldWidth();
		float height = gVars->pRenderer->GetWorldHeight();

		SRandomPolyParams params;
		params.minRadius = m_scale;
		params.maxRadius = m_scale;
		params.minBounds = Vec2(-width * 0.5f + params.maxRadius, -height * 0.5f + params.maxRadius);
		params.maxBounds = params.minBounds * -1.0f;
		params.minPoints = 3;
		params.maxPoints = 8;
		params.minSpeed = 1.0f;
		params.maxSpeed = 3.0f;

		for (size_t i = 0; i < m_polyCount; ++i)
		{
			gVars->pWorld->AddRandomPoly(params);
		}
	}

private:
	size_t m_polyCount;
	float m_scale;
};

#endif