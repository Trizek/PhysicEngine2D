#include "BroadPhaseSweepAndPrune.h"

#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

void CBroadPhaseSweepAndPrune::GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck)
{
	gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
	{
		poly->UpdateAABB();
	});

	if (m_polysXAxis.size() == 0)
	{
		gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
		{
			m_polysXAxis.push_back(poly);
		});
	}

	int i = (int)m_polysXAxis.size() - 2;
	while (i >= 0)
	{
		CPolygonPtr polyA = m_polysXAxis[i];

		// insert sort polyA in sorted list [i+1, size[
		int j = i + 1;
		while (j < (int)m_polysXAxis.size() && (polyA->aabb.min.x > m_polysXAxis[j]->aabb.min.x))
		{
			m_polysXAxis[j - 1] = m_polysXAxis[j];
			m_polysXAxis[j] = polyA;
			++j;
		}
		// polyA is located now at j - 1 index, and j verify (polyA->aabb.minx.x <= m_polysXAxis[j]->aabb.min.x)
		while (j < (int)m_polysXAxis.size() && (polyA->aabb.max.x > m_polysXAxis[j]->aabb.min.x))
		{
			// x colliding
			size_t indexA = polyA->GetIndex();
			size_t indexB = m_polysXAxis[j]->GetIndex();
			size_t key = Min(indexA, indexB) * m_polysXAxis.size() + Max(indexA, indexB);
			//m_collidingPairsOnX[key] = true;

			AABB& aabb1 = polyA->aabb;
			AABB& aabb2 = m_polysXAxis[j]->aabb;

			bool sep = (aabb1.max.y < aabb2.min.y) || (aabb2.max.y < aabb1.min.y);
			if (!sep && (polyA->density > 0.0f || m_polysXAxis[j]->density > 0.0f))
			{
				pairsToCheck.push_back(SPolygonPair(polyA, m_polysXAxis[j]));
			}

			++j;
		}

		--i;
	}
}