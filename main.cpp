// CollisionEngine.cpp : définit le point d'entrée pour l'application console.
//

#include "stdafx.h"

#include "Application.h"

#include "SceneManager.h"
#include "SceneFluid.h"


#include <iostream>
#include <string>




/**/



//#include <stdlib.h>
//class CMyScene : public IScene
//{
//public:
//
//	float Random(float from, float to)
//	{
//		return from + (to - from) * (((float)rand()) / ((float)RAND_MAX));
//	}
//
//	virtual void Create() override
//	{
//
//		float groundHeight = -0.5f * gVars->pRenderer->GetWorldHeight();
//
//		/*
//		{
//			CPolygonPtr sqr = gVars->pWorld->AddRectangle(gVars->pRenderer->GetWorldWidth(), 1.0f);
//			Vec2& pos = sqr->position;
//			pos.y = -gVars->pRenderer->GetWorldHeight() * 0.5f;
//			sqr->density = 0.0f;
//		}
//		{
//			CPolygonPtr sqr = gVars->pWorld->AddRectangle(1.0f, gVars->pRenderer->GetWorldHeight());
//			Vec2& pos = sqr->position;
//			pos.x = -gVars->pRenderer->GetWorldWidth() * 0.5f;
//			//pos.y = gVars->pRenderer->GetWorldHeight() * 0.5f;
//			sqr->density = 0.0f;
//		}
//		{
//			CPolygonPtr sqr = gVars->pWorld->AddRectangle(1.0f, gVars->pRenderer->GetWorldHeight());
//			Vec2& pos = sqr->position;
//			pos.x = gVars->pRenderer->GetWorldWidth() * 0.5f;
//			//pos.y = gVars->pRenderer->GetWorldHeight() * 0.5f;
//			sqr->density = 0.0f;
//		}
//
//		
//		float scale = 1.0f;
//
//		gVars->pRenderer->SetWorldHeight(60.0f); // 10.0f);
//
//		float width = gVars->pRenderer->GetWorldWidth();
//		float height = gVars->pRenderer->GetWorldHeight();
//
//		float polySpacing = 8.0f;
//
//		int polyCount = (width / polySpacing) * (height / polySpacing);
//
//		for (int i = 0; i < polyCount; ++i)
//		{
//			CPolygonPtr poly = gVars->pWorld->AddPolygon();
//			poly->points.push_back({ -1.0f, -1.5f });
//			poly->points.push_back({ 1.0f, -1.5f });
//			poly->points.push_back({ 1.5f, 0.0f });
//			poly->points.push_back({ 1.0f, 1.5f });
//			poly->points.push_back({ -1.0f, 1.0f });
//			poly->points.push_back({ -1.5f, 0.0f });
//			
//			poly->Build();
//			poly->rotation.SetAngle( Random(-180.0f, 180.0f) );
//			poly->position.x = Random(-width * 0.5f, width * 0.5f);
//			poly->position.y = Random(-height * 0.5f, height * 0.5f);
//
//			Mat2 rot;
//			rot.SetAngle(Random(-180.0f, 180.0f));
//			poly->speed = rot.X * 3.0f;
//		}
//
//		//gVars->pWorld->AddBehavior<CMovePolyBhv>(nullptr);
//
//		return;
//		
//		/*for (float x = -0.5f * gVars->pRenderer->GetWorldWidth(); x < 0.5f * gVars->pRenderer->GetWorldWidth(); x += 1.0f)
//		{
//			CPolygonPtr sqr = gVars->pWorld->AddSquare(1.0f);
//			Vec2& pos = sqr->position;
//			pos.x = x;
//			pos.y = groundHeight + 0.5f;	
//			sqr->density = 0.0f;
//		}*/
//
//
//
//		CPolygonPtr block = gVars->pWorld->AddSquare(20.0f);
//		block->position.x = 5.0f;
//		block->position.y = groundHeight + 10.0f;
//		block->density = 0.0f;
//
//		//CPolygonPtr triangle = gVars->pWorld->AddTriangle(10.0f, 15.0f);
//		//gVars->pWorld->AddBehavior<CRotateBehavior>(triangle);
//
//		
//		CPolygonPtr square = gVars->pWorld->AddSquare(15.0f);
//		//square->rotation.Rotate(40.0f);
//		square->position.x = 20.0f;
//		square->position.y += 10.0f;
//		//square->density = 0.0f;
//		
//		
//		CPolygonPtr poly = gVars->pWorld->AddPolygon();
//		poly->points.push_back({ -10.0f, -10.0f });
//		poly->points.push_back({ 10.0f, -10.0f });
//		poly->points.push_back({ 15.0f, 0.0f });
//		poly->points.push_back({ 10.0f, 10.0f });
//		poly->points.push_back({ -10.0f, 10.0f });
//		poly->points.push_back({ -15.0f, 0.0f });
//
//		for (Vec2& point : poly->points)
//		{
//			point /= 3.0f;
//		}
//
//		poly->Build();
//		poly->position.x = -10.0f; // -20.0f;
//		poly->position.y = 10.0f;
//		//poly->rotation.Rotate(-5.0f);
//
//		//poly->density = 0.0f;
//
//		/*CBehaviorPtr bhv = gVars->pWorld->AddBehavior<CShowDistanceBehavior>(nullptr);
//		static_cast<CShowDistanceBehavior*>(bhv.get())->polyA = poly;
//		static_cast<CShowDistanceBehavior*>(bhv.get())->polyB = square;
//		*/
//		gVars->pWorld->AddBehavior<CPolygonMoveTool>(nullptr);
//	}
//};
//
//class CMyScene2 : public IScene
//{
//public:
//
//
//	virtual void Create() override
//	{
//		CPolygonPtr poly = gVars->pWorld->AddTriangle(20.0f, 20.0f);
//	
//		Vec2 force(0.0f, -10.0f);
//		Vec2 forcePoint = poly->position + Vec2(-5.0f, 0.0f);
//
//		poly->torques = (forcePoint - poly->position) ^ force;
//	}
//};
//
//

//#include "InertiaTensor.h"

#include "Scenes/SceneDebugCollisions.h"
#include "Scenes/SceneBouncingPolys.h"
#include "Scenes/SceneSimplePhysic.h"
#include "Scenes/SceneSpheres.h"
#include "Scenes/SceneComplexPhysic.h"
#include "Scenes/SceneSmallPhysic.h"


/*
* Entry point
*/
int _tmain(int argc, char** argv)
{


	InitApplication(1260, 768, 50.0f);

	
	//gVars->pSceneManager->AddScene(new CSceneDebugCollisions());
	//gVars->pSceneManager->AddScene(new CSceneBouncingPolys(200));
	//gVars->pSceneManager->AddScene(new CSceneSpheres());
	//gVars->pSceneManager->AddScene(new CSceneSimplePhysic());
	//gVars->pSceneManager->AddScene(new CSceneComplexPhysic(25));

	gVars->pSceneManager->AddScene(new CSceneFluid());
	gVars->pSceneManager->AddScene(new CSceneSmallPhysic());
	gVars->pSceneManager->AddScene(new CSceneSimplePhysic());
	gVars->pSceneManager->AddScene(new CSceneComplexPhysic(25));

	/*
	gVars->pSceneManager->AddScene(new CSceneDebugCollisions());
	gVars->pSceneManager->AddScene(new CSceneSmallPhysic());
	gVars->pSceneManager->AddScene(new CSceneSimplePhysic());
	gVars->pSceneManager->AddScene(new CSceneComplexPhysic(25));*/

	/*
	gVars->pSceneManager->AddScene(new CMyScene());
	gVars->pSceneManager->AddScene(new CMyScene2());
	*/

	RunApplication();
	return 0;
}

