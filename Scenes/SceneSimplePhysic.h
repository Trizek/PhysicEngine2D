#ifndef _SCENE_SIMPLE_PHYSIC_H_
#define _SCENE_SIMPLE_PHYSIC_H_

#include "BaseScene.h"


class CSceneSimplePhysic: public CBaseScene
{
public:
	CSceneSimplePhysic(float scale = 1.0f) : CBaseScene(0.5f * scale, 10.0f * scale), m_scale(scale){}

private:
	virtual void Create() override
	{
		CBaseScene::Create();

		float coeff = m_scale * 0.2f;

		CPolygonPtr block = gVars->pWorld->AddRectangle(coeff * 13.0f, coeff * 15.0f);
		block->position = Vec2(0.0f, -coeff * 7.0f);
		block->density = 0.0f;

		CPolygonPtr rectangle = gVars->pWorld->AddRectangle(coeff * 30.0f, coeff * 10.0f);
		rectangle->position = Vec2(coeff * 15.0f, coeff * 5.0f);

		for (int i = 0; i < 4; ++i)
		{

			CPolygonPtr sqr = gVars->pWorld->AddSquare(coeff * 10.0f);
			sqr->density = 0.5f;
			sqr->position = Vec2(coeff * 15.0f, coeff * 15.0f);
		}
		CPolygonPtr tri = gVars->pWorld->AddTriangle(coeff * 5.0f, coeff * 5.0f);
		tri->position = Vec2(coeff * 5.0f, coeff * 15.0f);
		tri->density *= 5.0f;
		//
		gVars->pWorld->AddSymetricPolygon(coeff * 10.0f, 50)->position = Vec2(-coeff * 20.0f, coeff * 5.0f);
	}

	float m_scale;
};

#endif