#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <ppl.h>

#include "WorldMap.h"
#include "EightPoint.h"
#include "LineSegmentWrapper.h"
#include "Homography.h"
#include "WorldMapView.h"

#include "mycv.h"
#include "opencv2/highgui.hpp"
#include "opencv2/line_descriptor.hpp"

class CMapManager
{
private:
	CWorldMapView*	m_pWorldMapView;

	CWorldMap*	m_pWorldMap;
	CSLAMKeyFrame*	m_pCurrKeyFrame;

	boost::mutex	m_mutexKeyFrameQueue;
	std::queue<CSLAMKeyFrame *>	m_qKeyFrames;

	boost::mutex	m_mutexStop;
	bool	m_bStop;

	CLineSegmentWrapper	m_lsdWrapper;
public:
	CMapManager(CWorldMap* pWorldMap/*, CWorldMapView* pWorldMapView*/);
	CMapManager();
	~CMapManager();

	void	SetWorldMap(CWorldMap* pWorldMap);
	void	SetWorldMapView(CWorldMapView* pWorldMapView);

	void	Run(void);

	void	AddMapPlane(CMapPlane* pNewMapPlane) {
		m_pWorldMap->AddMapPlane(pNewMapPlane);
	}

	void	AddMapLine(CMapLine* pNewMapLine) {
		m_pWorldMap->AddMapLine(pNewMapLine);
	}

	bool	IsNewKeyFrame(void);

	void	SetStopFlag(bool bStop);
	bool	IsStop(void);

	void	Clear(void);

	void	AddMapPoint(CMapPoint* pNewMapPoint);
	void	AddKeyFrame(CSLAMKeyFrame* pNewKeyFrame);
	void	AddInitKeyFrame(CSLAMKeyFrame* pNewKeyFrame);
	std::set<CMapPlane*>	GetMapPlanes(void) {
		return m_pWorldMap->GetMapPlanes();
	}

	void	ProcessMapPoints(void);
	void	ProcessMapLines(void);
	void	ProcessMapPlanes(void);
	void	ManageMapPlanes(void);

	void	DrawCurrentKeyFrame(void);
private:
	std::vector<int>	FindEpipolarGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1,
															  const std::vector<SiftKeyPoint>& vKeyPoints2,
															  const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair);

	void	FindPlanarTransformMatches(const std::vector<SiftKeyPoint>& vKeyPoints1,
									   const std::vector<SiftKeyPoint>& vKeyPoints2,
									   const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
									   std::vector<int>& vHInliers, std::vector<int>& vHOutliers);

	void	EstimateNewMapPlanes(std::vector<CMapPoint*>& vMapPointsOnPlane, arma::vec& abcd, std::vector<int>& vInliers);
	void	ProcessTrackedMapPlanes(std::set<CMapPoint*>& sMapPointsOnPlane, std::map<CMapPlane*, int>& mMapPointCountOnPlanes);
};