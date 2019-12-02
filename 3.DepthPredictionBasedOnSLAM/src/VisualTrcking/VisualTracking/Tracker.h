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

#include "mycv.h"
#include "opencv2/highgui.hpp"

class CTracker
{
private:
	arma::mat	K;
	arma::vec	D;

	int	m_nCurrentFrameNumber;
	int	m_nCurrentState;
	CImageGrabber*	m_pImageGrabber;
	CMapManager*	m_pMapManager;
	CFrame	m_currFrame;
	CFrame	m_lastFrame;
	CKeyFrame*	m_pCurrKeyFrame;
public:
	CTracker(CImageGrabber* pImageGrabber, CMapManager* pMapMaker, const arma::mat& Intrinsic, const arma::vec& DistCoeff);
	CTracker();
	~CTracker();

	void	Track(void);
private:
	bool	TrackFromPrevisouFrame(void);
	void	CheckNewKeyFrame(void);

	void	VisualizeCurrentFrame(void);

	void	InitializeVisualTracking(void);

	std::vector<int>	FindGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1, 
													  const std::vector<SiftKeyPoint>& vKeyPoints2,
													  const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair);
};

