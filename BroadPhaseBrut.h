#ifndef _BROAD_PHASE_BRUT_H_
#define _BROAD_PHASE_BRUT_H_

#include "BroadPhase.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

class CBroadPhaseBrut : public IBroadPhase
{
public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		for (size_t i = 0; i < gVars->pWorld->GetPolygonCount(); ++i)
		{
			for (size_t j = i + 1; j < gVars->pWorld->GetPolygonCount(); ++j)
			{
				CPolygonPtr pA = gVars->pWorld->GetPolygon(i);
				CPolygonPtr pB = gVars->pWorld->GetPolygon(j);
				
				if (pA->density == 0.0f && pB->density == 0.0f)
					continue;

				pairsToCheck.push_back(SPolygonPair(gVars->pWorld->GetPolygon(i), gVars->pWorld->GetPolygon(j)));
			}
		}
	}
};

#endif