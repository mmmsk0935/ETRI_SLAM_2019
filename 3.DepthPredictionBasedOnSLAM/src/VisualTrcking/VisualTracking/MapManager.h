#pragma once

#include <queue>

#include "WorldMap.h"
#include "EightPoint.h"

class CMapManager
{
private:
	CWorldMap*	m_pWorldMap;
	CKeyFrame*	m_pCurrKeyFrame;
	std::queue<CKeyFrame *>	m_qKeyFrames;

public:
	CMapManager(CWorldMap* pWorldMap);
	~CMapManager();

	void	Clear(void);

	void	AddMapPoint(CMapPoint* pNewMapPoint);
	void	AddKeyFrame(CKeyFrame* pNewKeyFrame);

	void	ProcessNewKeyFrame(void);
	void	ProcessMapPlanes(void);

private:
	std::vector<int>	FindEpipolarGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1,
															  const std::vector<SiftKeyPoint>& vKeyPoints2,
															  const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair);
};