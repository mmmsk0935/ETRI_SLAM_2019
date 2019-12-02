#pragma once

#include "armadillo"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "VisualSLAMConfigure.h"
#include "KeyFrame.h"

class CKeyFrame;
class CFrame;
class CMapPoint
{
private:
	boost::mutex	m_mutexPos;
	arma::vec	X;

	boost::mutex	m_mutexVariance;
	double	m_dbAngleVariance;

	boost::mutex	m_mutexKeyFrames;
	std::map<CKeyFrame*, int>	m_mObservedKeyFrame;
public:
	CMapPoint(WorldPoint pos);
	CMapPoint(const arma::vec& pos);
	~CMapPoint();

	void	SetPosition(const arma::vec& pos);
	
	arma::vec	GetPosition(void);
	
	std::map<CKeyFrame*, int>	GetObservedKeyFrames();

	void	AddObservedKeyFrame(CKeyFrame *pKeyFrame, int nPointFeatureIdx);
};

