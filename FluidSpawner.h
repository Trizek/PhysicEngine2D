#ifndef _FLUID_SPAWNER_H_
#define _FLUID_SPAWNER_H_

#include "Behavior.h"
#include "FluidSystem.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "World.h"


class CFluidSpawner: public CBehavior
{
private:
	virtual void Update(float frameTime) override
	{
		bool clicking = gVars->pRenderWindow->GetMouseButton(0);
		if (!m_clicking && clicking)
		{
			Vec2 mousePoint = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());
			gVars->pFluidSystem->Spawn(mousePoint - Vec2(0.5f, 0.5f), mousePoint + Vec2(0.5f, 0.5f), 10.0f, Vec2(15.0f, 15.0f));
		//	gVars->pFluidSystem->SpawnParticule(mousePoint, Vec2());// Vec2(15.0f, 15.0f));
		}
		m_clicking = clicking;
	}

	bool m_clicking = false;
};


#endif