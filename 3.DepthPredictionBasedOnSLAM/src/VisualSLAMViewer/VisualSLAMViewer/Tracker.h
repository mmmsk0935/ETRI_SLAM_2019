#pragma once

#include <iostream>
#include <vector>
#include <ppl.h>

#include "ImageGrabber.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapManager.h"
#include "EightPoint.h"
#include "RigidBody.h"
#include "Homography.h"
#include "Optimizer.h"
#include "WorldMapView.h"
#include "PlaneEstimator.h"
#include "Projective.h"

#include "mycv.h"
#include "opencv2/highgui.hpp"

class CTracker
{
private:
	arma::mat	K;
	arma::vec	D;

	int	m_nCurrentFrameNumber;

	boost::mutex	m_muCurrentState;
	int	m_nCurrentState;
	CImageGrabber*	m_pImageGrabber;
	CMapManager*	m_pMapManager;
	CFrame	m_initFrame;
	CFrame	m_currFrame;
	CFrame	m_lastFrame;
	CSLAMKeyFrame*	m_pPrevKeyFrame;
	CSLAMKeyFrame*	m_pCurrKeyFrame;

	CWorldMapView*	m_pWorldMapView;

	std::vector<std::pair<SiftKeyPoint, SiftKeyPoint>>	m_vTrackForInitialization;

	CLineSegmentWrapper	m_lsdWrapper;
public:
	CTracker(CImageGrabber* pImageGrabber, CMapManager* pMapMaker, CWorldMapView* pWorldMapView, const arma::mat& Intrinsic, const arma::vec& DistCoeff);
	CTracker();
	~CTracker();

	void	PrepaerTracking(CImageGrabber* pImageGrabber, CMapManager* pMapMaker, CWorldMapView* pWorldMapView, const arma::mat& Intrinsic, const arma::vec& DistCoeff);
	void	Track(void);

	void	SetCurrentState(int nCurrentState) {
		boost::mutex::scoped_lock lock(m_muCurrentState);
		m_nCurrentState = nCurrentState;
	}

	void	Reset(void) {
		m_pCurrKeyFrame = nullptr;
		m_pPrevKeyFrame = nullptr;
		SetCurrentState(TrackingState::Idle);
	}
private:
	bool	TrackFromPrevisouFrame(void);
	bool	TrackFromCurrKeyFrame(void);

	void	CheckNewKeyFrame(void);

	void	VisualizeCurrentFrame(void);

	void	InitializeForRGBD(void);

	void	FindGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1, 
										  const std::vector<SiftKeyPoint>& vKeyPoints2,
										  const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
										  std::vector<int>& vInlierIndices, arma::mat& F);

	void	FindPlanarMatchFromView(const std::vector<SiftKeyPoint>& vKeyPoints1, 
									const std::vector<SiftKeyPoint>& vKeyPoints2,
									const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
									std::vector<int>& vInlierIndices, arma::mat& H);

	int		GetCurrentState(void) {
		boost::mutex::scoped_lock lock(m_muCurrentState);
		return m_nCurrentState;
	}

	std::vector<CMapPlane*>	EstimateInitialMapPlane(std::vector<CMapPoint *>& vMapPoints);

	void	InitializingForMonocular(void);

	bool	ReconstructTwoViewFromH(arma::mat& H, std::vector<std::pair<int, int>>& vCoarseMatchPair, std::vector<int>& vInliers);
	bool	ReconstructTwoViewFromF(arma::mat& F, std::vector<std::pair<int, int>>& vCoarseMatchPair, std::vector<int>& vInliers);

	void	UpdateCurrentKeyFrame(void);
};