#ifndef _BROAD_PHASE_SWEEP_H_
#define _BROAD_PHASE_SWEEP_H_

#include "BroadPhase.h"

class CBroadPhaseSweepAndPrune : public IBroadPhase
{
public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override;

private:
	// Broad phase
	std::vector<CPolygonPtr>			m_polysXAxis;
	std::unordered_map<size_t, bool>	m_collidingPairsOnX;
};

#endif