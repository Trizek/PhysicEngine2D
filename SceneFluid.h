#ifndef _SCENE_FLUID_H_
#define _SCENE_FLUID_H_

#include "Scenes\BaseScene.h"

#include "FluidSpawner.h"

class CSceneFluid : public CBaseScene
{
public:
	CSceneFluid() : CBaseScene(1.0f, 10.0f){}

protected:


	virtual void Create() override
	{
		CBaseScene::Create();

		gVars->pWorld->AddBehavior<CFluidSpawner>(nullptr);
	}
};

#endif